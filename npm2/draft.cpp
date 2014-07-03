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

#include <npm2/DifferentialTrailerDrive.hpp>
#include <npm2/RayDistanceSensor.hpp>
#include <npm2/RevoluteServo.hpp>
#include <npm2/gfx.hpp>
#include <npm2/Simulator.hpp>
#include <npm2/Factory.hpp>
#include <npm2/Bob.hpp>
#include <sfl/util/numeric.hpp>
#include <iostream>

#include <cmath>
#include <stdio.h>
#include <err.h>


using namespace npm2;


static Simulator * simulator (0);
static Bob * bob (0);

static double mx0, my0, mx1, my1;


static void recurse_draw (Object const * obj)
{
  gfx::set_pen (2.0, 0.0, 0.0, 0.0, 1.0);
  Body::lines_t const & lines (obj->body_.getLines());
  for (size_t il(0); il < lines.size(); ++il) {
    gfx::draw_line (lines[il].X0(), lines[il].Y0(), lines[il].X1(), lines[il].Y1());
  }
  
  RayDistanceSensor const * rds (dynamic_cast <RayDistanceSensor const *> (obj));
  if (rds) {
    gfx::set_pen (1.0, 1.0, 0.0, 0.0, 1.0);
    gfx::draw_line (rds->getGlobal().X(),
		    rds->getGlobal().Y(),
		    rds->getGlobal().X() + rds->distance_ * rds->getGlobal().Costheta(),
		    rds->getGlobal().Y() + rds->distance_ * rds->getGlobal().Sintheta());
  }
  
  for (Object::child_iterator_t ic(obj->childBegin()); ic != obj->childEnd(); ++ic) {
    recurse_draw (*ic);
  }
}


static void cb_draw ()
{
  BBox const & bbox (simulator->world_->getBBox());
  if ( ! bbox.isValid()) {
    return;
  }
  
  // world
  
  static double const margin (0.1);
  gfx::set_view (bbox.x0() - margin, bbox.y0() - margin, bbox.x1() + margin, bbox.y1() + margin);
  recurse_draw (simulator->world_);
  
  // bob drive: should now be handled via some standardized object drawing scheme
  
  gfx::set_pen (2.0, 0.0, 0.5, 0.5, 1.0);
  double x0, y0, x1, y1;
  x0 =  0.0;
  y0 = -bob->drive_->wheel_base_ / 2.0;
  x1 =  0.0;
  y1 = -y0;
  bob->drive_->getParent()->getGlobal().To (x0, y0);
  bob->drive_->getParent()->getGlobal().To (x1, y1);
  gfx::draw_line (x0, y0, x1, y1);
  x0 = -bob->drive_->wheel_radius_;
  y0 =  bob->drive_->wheel_base_ / 2.0;
  x1 = -x0;
  y1 =  y0;
  bob->drive_->getParent()->getGlobal().To (x0, y0);
  bob->drive_->getParent()->getGlobal().To (x1, y1);
  gfx::draw_line (x0, y0, x1, y1);
  x0 = -bob->drive_->wheel_radius_;
  y0 = -bob->drive_->wheel_base_ / 2.0;
  x1 = -x0;
  y1 =  y0;
  bob->drive_->getParent()->getGlobal().To (x0, y0);
  bob->drive_->getParent()->getGlobal().To (x1, y1);
  gfx::draw_line (x0, y0, x1, y1);
  x0 = -bob->drive_->hitch_offset_;
  y0 =  0.0;
  x1 =  0.0;
  y1 =  0.0;
  bob->drive_->getParent()->getGlobal().To (x0, y0);
  bob->drive_->getParent()->getGlobal().To (x1, y1);
  gfx::draw_line (x0, y0, x1, y1);
  x0 = -bob->drive_->hitch_offset_;
  y0 =  0.0;
  x1 =  x0 - bob->drive_->trailer_arm_ * cos (bob->drive_->getTrailerAngle());
  y1 =     - bob->drive_->trailer_arm_ * sin (bob->drive_->getTrailerAngle());
  bob->drive_->getParent()->getGlobal().To (x0, y0);
  bob->drive_->getParent()->getGlobal().To (x1, y1);
  gfx::draw_line (x0, y0, x1, y1);

  gfx::set_pen (2.0, 0.0, 0.6, 0.4, 1.0);
  x0 =  0.0;
  y0 = -bob->drive_->wheel_base_ / 2.0;
  x1 =  0.0;
  y1 = -y0;
  bob->drive_->trailer_->getGlobal().To (x0, y0);
  bob->drive_->trailer_->getGlobal().To (x1, y1);
  gfx::draw_line (x0, y0, x1, y1);
  x0 = -bob->drive_->wheel_radius_;
  y0 =  bob->drive_->wheel_base_ / 2.0;
  x1 = -x0;
  y1 =  y0;
  bob->drive_->trailer_->getGlobal().To (x0, y0);
  bob->drive_->trailer_->getGlobal().To (x1, y1);
  gfx::draw_line (x0, y0, x1, y1);
  x0 = -bob->drive_->wheel_radius_;
  y0 = -bob->drive_->wheel_base_ / 2.0;
  x1 = -x0;
  y1 =  y0;
  bob->drive_->trailer_->getGlobal().To (x0, y0);
  bob->drive_->trailer_->getGlobal().To (x1, y1);
  gfx::draw_line (x0, y0, x1, y1);
  
}


static void cb_mouse (double mx, double my, int flags)
{
  if (flags & gfx::MOUSE_PRESS) {
    mx0 = mx;
    my0 = my;
  }
  else if (flags & gfx::MOUSE_DRAG) {
    mx1 = mx;
    my1 = my;
  }
}


static void tick ()
{
  simulator->simulateActuators ();
  simulator->simulateSensors ();
  simulator->simulateProcesses ();
}


static void cb_idle ()
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
}


static void cb_pause ()
{
  if (simulator->state_ == Simulator::RUN) {
    simulator->state_ = Simulator::PAUSE;
  }
  else {
    simulator->state_ = Simulator::RUN;
  }
}


static void cb_next ()
{
  simulator->state_ = Simulator::STEP;
}


static void parse_cfile (char const * cfname)
{
  npm2::Factory & ff (npm2::Factory::instance());
  if ( ! ff.parseFile (cfname, &cerr)) {
    errx (EXIT_FAILURE, "%s: parse error (see above messages)", cfname);
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
	      "  Copyright (C) 2014 Roland Philippsen. All rights reserved.\n"
	      "  Released under the terms of the GNU General Public License.\n"
	      "\n"
	      "  -h        print this help message.\n"
	      "  -c cfile  read additional configuration files.\n",
	      argv[0]);
    }
    else {
      parse_cfile (argv[ii]);
      config_ok = true;
    }
  }
  
  if ( ! config_ok) {
    errx (EXIT_FAILURE, "expected configuration file");
  }
}


int main (int argc, char ** argv)
{
  parse_args (argc, argv);
  
  if (1 != npm2::Factory::instance().findRegistry <Simulator> ()->size()) {
    errx (EXIT_FAILURE, "exactly one Simulator needs to be configured (for now)");
  }
  simulator = npm2::Factory::instance().findRegistry <Simulator> ()->at (0);
  if ( ! simulator->world_) {
    errx (EXIT_FAILURE, "no world given to simulator");
  }
  
  bob = npm2::Factory::instance().find <Bob> ("bob");
  if ( ! bob) {
    errx (EXIT_FAILURE, "cannot find bob");
  }
  
  // Make sure we can draw something before we start running.
  //
  simulator->world_->updateTransform ();
  
  //////////////////////////////////////////////////
  
  mx0 =  0.0;
  my0 =  0.0;
  mx1 =  0.5;
  my1 = -0.5;
  
  // // This just enables (rather verbose) debug messages from the gfx
  // // wrapper.  It is optional but can be rather useful.
  // gfx::debug (&cout);
  
  // Adding a custom button needs to happen before gfx::main is called.
  gfx::add_button ("run/pause", cb_pause);
  
  // Adding a custom button needs to happen before gfx::main is called.
  gfx::add_button ("next", cb_next);
  
  // The gfx::main function enters the GUI processing loop.  It
  // returns when the user has clicked the "quit" button (which gets
  // added automatically) or otherwise exited the GUI application.
  gfx::main (argv[0], cb_idle, cb_draw, cb_mouse);
}
