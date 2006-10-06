/* 
 * Copyright (C) 2004
 * Swiss Federal Institute of Technology, Lausanne. All rights reserved.
 * 
 * Author: Roland Philippsen <roland dot philippsen at gmx dot net>
 *         Autonomous Systems Lab <http://asl.epfl.ch/>
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


#include "Simulator.hpp"
#include "Interlock.hpp"
#include <npm/common/Globals.hpp>
#include <npm/common/World.hpp>
#include <npm/common/wrap_glut.hpp>
#include <sfl/util/Pthread.hpp>
#include <iostream>


using namespace npm;
using namespace sfl;
using namespace boost;
using namespace std;


class Parameters
{
public:
  Parameters():
    robot_config_filename("robots.config"),
    layout_config_filename("layout.config"),
    world_name("stage"),
    threaded(false)
  {
  }
  
  string robot_config_filename;
  string layout_config_filename;
  string world_name;
  bool threaded;
};


static Parameters params;
static const unsigned int glut_timer_ms(50);
static const unsigned int timestep_usec(100000);
static int handle;
static shared_ptr<Simulator> simulator;
static shared_ptr<SimulatorUpdateThread> update_thread;

static void parse_options(int argc, char ** argv);
static void init_glut(int argc, char** argv, int width, int height);
static void reshape(int width, int height);
static void draw();
static void keyboard(unsigned char key, int x, int y);
static void timer(int handle);
static void cleanup();


int main(int argc, char ** argv)
{
  ////  atexit(cleanup);
  parse_options(argc, argv);

  shared_ptr<World> world(World::Create(params.world_name));
  if( ! world){
    cerr << "Invalid World \"" << params.world_name << "\"\n";
    exit(EXIT_FAILURE);
  }
  
  simulator.
    reset(new Simulator(world, 0.000001 * timestep_usec, SimulatorMutex()));
  simulator->InitRobots(params.robot_config_filename);
  simulator->InitLayout(params.layout_config_filename);
  simulator->Init();
  
  if(params.threaded){
    update_thread.reset(new SimulatorUpdateThread("nepumuk", simulator));
    update_thread->Start(timestep_usec);
  }
  
  init_glut(argc, argv, 700, 500);
  glutMainLoop();
}


void init_glut(int argc, char ** argv, int width, int height)
{
  simulator->Reshape(width, height);
  
  glutInit(&argc, argv);
  glutInitDisplayMode(GLUT_RGBA | GLUT_DOUBLE);
  glutInitWindowPosition(0, 0);
  glutInitWindowSize(width, height);

  handle = glutCreateWindow("nepumuk");
  if(handle == 0){
    cerr << "init_glut(): glutCreateWindow() failed\n";
    exit(EXIT_FAILURE);
  }
  
  glutDisplayFunc(draw);
  glutReshapeFunc(reshape);
  glutTimerFunc(glut_timer_ms, timer, handle);
  glutKeyboardFunc(keyboard);
}


void reshape(int width, int height)
{
  simulator->Reshape(width, height);
}


void draw()
{
  glClear(GL_COLOR_BUFFER_BIT);
  simulator->Draw();
  glFlush();
  glutSwapBuffers();
}


void keyboard(unsigned char key,
	      int x,
	      int y)
{
  simulator->Keyboard(key, x, y);
}


void timer(int handle)
{
  if((update_thread && update_thread->Changed())
      || simulator->Idle()){
    glutSetWindow(handle);
    glutPostRedisplay();
  }
  glutTimerFunc(glut_timer_ms, timer, handle);
}


void parse_options(int argc,
		   char ** argv)
{
  typedef Interlock::Callback<string> StringCB;

  Interlock ilock;
  ilock.Add(new StringCB(params.layout_config_filename,
			 'l', "layout", "Name of the layout config file."));
  ilock.Add(new StringCB(params.robot_config_filename,
			 'r', "robot", "Name of the robot config file."));
  ilock.Add(new StringCB(params.world_name,
			 'w', "world", "Name of the world (expo|mini|tta)."));
  ilock.
    Add(new Interlock::BoolCallback(params.threaded, 't', "threaded",
				    "Use separate GLUT thread."));
  
  ilock.UsageMessage(cerr, string(argv[0]) + " <options>");
  try {
    ilock.Parse(argc, argv, 0);
  }
  catch(runtime_error e){
    cerr << "ERROR in parse_options():\n  " << e.what() << "\n";
    exit(EXIT_FAILURE);
  }
}


void cleanup()
{
  cerr << "cleanup: resetting update_thread\n";
  update_thread.reset();
  cerr << "cleanup: resetting simulator\n";
  simulator.reset();
}
