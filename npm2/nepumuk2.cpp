/* Nepumuk Mobile Robot Simulator v2
 *
 * Copyright (C) 2014 Roland Philippsen. All rights reserved.
 * 
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 * 
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307
 * USA
 */

#include <npm2/View.hpp>
#include <npm2/Simulator.hpp>
#include <npm2/Factory.hpp>
#include <npm2/Object.hpp>
#include <err.h>

// tmp
#include <npm2/RayDistanceSensor.hpp>
#include <npm2/Drawing.hpp>
#include <npm2/Camera.hpp>
#include <npm2/gl.hpp>

using namespace npm2;


static Simulator * simulator (0);
static int window_handle;
static unsigned int const glut_timer_ms (1);
static View view ("tmp");


namespace {
  
  class tmpDrawing
    : public Drawing
  {
  public:
    tmpDrawing (): Drawing ("tmp", "tmp") {}
    virtual void draw ();
  };
  
  class tmpCamera
    : public Camera
  {
  public:
    tmpCamera (): Camera ("tmp", "tmp") {}
    virtual void configureView (View & view);
  };
  
}


static void parse_cfile (char const * cfname)
{
  npm2::Factory & ff (npm2::Factory::instance());
  if ( ! ff.parseFile (cfname, &cerr)) {
    errx (EXIT_FAILURE, "%s: parse error (see above messages)", cfname);
  }
  
  new tmpCamera ();
  if ( ! view.setCamera ("tmp")) {
    exit (EXIT_FAILURE);
  }
  
  new tmpDrawing ();
  if ( ! view.addDrawing ("tmp")) {
    exit (EXIT_FAILURE);
  }
}


static void parse_args (int argc, char ** argv)
{
  bool config_ok (false);
  
  for (int ii (1); ii < argc; ++ii) {
    if (0 == strcmp ("-c", argv[ii])) {
      if (++ii >= argc) {
	errx (EXIT_FAILURE, "-c option expects argument");
      }
      parse_cfile (argv[ii]);
      config_ok = true;
    }
    else if (0 == strcmp ("-h", argv[ii])) {
      printf ("%s cfile [-c cfile] [-h]\n"
	      "  Nepumuk Mobile Robot Simulator v2\n"
	      "  Copyrights (C) 2004-2014 by\n"
	      "    Swiss Federal Institute of Technology, Lausanne, and\n"
	      "    Roland Philippsen. All rights reserved.\n"
	      "  Released under the terms of the GNU General Public License.\n"
	      "\n"
	      "  -h        print this help message.\n"
	      "  -c cfile  read additional configuration files.\n",
	      argv[0]);
      exit (EXIT_SUCCESS);
    }
    else {
      parse_cfile (argv[ii]);
      config_ok = true;
    }
  }
  
  printf ("Nepumuk Mobile Robot Simulator v2\n"
	  "  Swiss Federal Institute of Technology, Lausanne, and Roland Philippsen.\n"
	  "  Released under the terms of the GNU General Public License.\n");
  
  if ( ! config_ok) {
    errx (EXIT_FAILURE, "expected configuration file");
  }
}


static void reshape (int width, int height)
{
  view.reshape (width, height);
}


static void tmp_recurse_draw (Object const * obj)
{
  glLineWidth (2);
  glColor3d (1.0, 1.0, 1.0);
  glBegin (GL_LINES);
  Body::lines_t const & lines (obj->body_.getLines());
  for (size_t il(0); il < lines.size(); ++il) {
    glVertex2d (lines[il].X0(), lines[il].Y0());
    glVertex2d (lines[il].X1(), lines[il].Y1());
  }
  glEnd ();
  
  RayDistanceSensor const * rds (dynamic_cast <RayDistanceSensor const *> (obj));
  if (rds) {
    glLineWidth (1);
    glColor3d (1.0, 0.0, 0.0);
    glBegin (GL_LINES);
    glVertex2d (rds->getGlobal().X(),
		rds->getGlobal().Y());
    glVertex2d (rds->getGlobal().X() + rds->distance_ * rds->getGlobal().Costheta(),
		rds->getGlobal().Y() + rds->distance_ * rds->getGlobal().Sintheta());
    glEnd ();
    glPointSize (3);
    glColor3d (1.0, 0.5, 0.0);
    glVertex2d (rds->getGlobal().X() + rds->distance_ * rds->getGlobal().Costheta(),
		rds->getGlobal().Y() + rds->distance_ * rds->getGlobal().Sintheta());
    glEnd();
  }
  
  for (Object::child_iterator_t ic(obj->childBegin()); ic != obj->childEnd(); ++ic) {
    tmp_recurse_draw (*ic);
  }
}


static void draw ()
{
  glutSetWindow (window_handle); // not sure this is needed...
  glClear (GL_COLOR_BUFFER_BIT);
  
  // for each view, draw its things ... later.
  view.draw ();
  
  glFlush ();
  glutSwapBuffers ();
}


static void keyboard (unsigned char key, int mx, int my)
{
  switch (key) {
  case ' ':
    simulator->state_ = Simulator::STEP;
    break;
  case 'c':
    if (simulator->state_ == Simulator::RUN) {
      simulator->state_ = Simulator::PAUSE;
    }
    else {
      simulator->state_ = Simulator::RUN;
    }
    break;
  case 'q':
    errx (EXIT_SUCCESS, "quit");
  }
  glutPostRedisplay ();
}


static void tick ()
{
  simulator->simulateActuators ();
  simulator->simulateSensors ();
  simulator->simulateProcesses ();
  
  glutSetWindow (window_handle); // needed?
  glutPostRedisplay ();
}


static void timer (int handle)
{
  switch (simulator->state_) {
  case Simulator::PAUSE:
    break;
  case Simulator::STEP:
    tick();
    simulator->state_ = Simulator::PAUSE;
    break;
  case Simulator::RUN:
  default:
    tick();
  }
  
  glutTimerFunc (glut_timer_ms, timer, handle);
}


static void init_glut (int argc, char ** argv)
{
  glutInit (&argc, argv);
  glutInitDisplayMode (GLUT_RGBA | GLUT_DOUBLE);
  
  glutInitWindowPosition (simulator->window_posx_, simulator->window_posy_);
  glutInitWindowSize (simulator->window_width_, simulator->window_height_);
  window_handle = glutCreateWindow (simulator->name.c_str());
  if (0 == window_handle) {
    errx(EXIT_FAILURE, "glutCreateWindow() failed");
  }
  
  glutDisplayFunc (draw);
  glutReshapeFunc (reshape);
  glutKeyboardFunc (keyboard);
  glutTimerFunc (glut_timer_ms, timer, window_handle);
}


int main (int argc, char ** argv)
{
  parse_args (argc, argv);
  
  // Should really add support for singletons to fpplib
  //
  if (1 != npm2::Factory::instance().findRegistry <Simulator> ()->size()) {
    errx (EXIT_FAILURE, "exactly one Simulator needs to be configured (for now)");
  }
  simulator = npm2::Factory::instance().findRegistry <Simulator> ()->at (0);
  if ( ! simulator->world_) {
    errx (EXIT_FAILURE, "no world given to simulator");
  }
  
  simulator->world_->updateTransform ();
  
  init_glut (argc, argv);
  glutMainLoop();
}


namespace {

  void tmpDrawing::draw ()
  {
    tmp_recurse_draw (simulator->world_);
  }
  
  void tmpCamera::configureView (View & view)
  {
    BBox const & bbox (simulator->world_->getBBox());
    if (bbox.isValid()) {
      static double const margin (0.1);
      view.setBounds (bbox, margin);
    }
    else {
      view.setBounds (0.0, 0.0, 1.0, 1.0);
    }
  }
  
}
