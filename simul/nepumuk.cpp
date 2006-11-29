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
#include <sfl/util/array2d.hpp>
#include <sfl/util/Line.hpp>
#include <sfl/gplan/GridFrame.hpp>
#include <iostream>
#include <fstream>
#include <sstream>
#include <signal.h>


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
    world_name(""),
    traversability_filename(""),
    //    threaded(false),
    no_glut(false),
    fatal_warnings(false)
  {
  }
  
  string robot_config_filename;
  string layout_config_filename;
  string world_name;
  string traversability_filename;
  //  bool threaded;
  bool no_glut;
  bool fatal_warnings;
};


struct travmap {
  double resolution, origin_x, origin_y, origin_theta;
  int freespace, obstacle;//, default;
  string worldname;
  shared_ptr<array2d<int> >  data;
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
static void sighandle(int signum);

static shared_ptr<travmap> parse_traversability(istream & is, ostream & os);
static void apply_traversability(shared_ptr<const travmap> travmap,
				 shared_ptr<World> world);


int main(int argc, char ** argv)
{
  atexit(cleanup);
  if(signal(SIGINT, sighandle) == SIG_ERR){
    perror("signal(SIGINT) failed");
    exit(EXIT_FAILURE);
  }
  if(signal(SIGHUP, sighandle) == SIG_ERR){
    perror("signal(SIGHUP) failed");
    exit(EXIT_FAILURE);
  }
  if(signal(SIGTERM, sighandle) == SIG_ERR){
    perror("signal(SIGTERM) failed");
    exit(EXIT_FAILURE);
  }
  
  parse_options(argc, argv);
  
  shared_ptr<World> world;
  shared_ptr<travmap> traversability;
  if( ! params.world_name.empty()){
    if( ! params.traversability_filename.empty()){
      cerr << "ERROR: specify either world name or traversability file.\n";
      exit(EXIT_FAILURE);
    }
    world = World::Create(params.world_name);
    if( ! world){
      cerr << "Invalid World \"" << params.world_name << "\"\n";
      exit(EXIT_FAILURE);
    }
  }
  else if( ! params.traversability_filename.empty()){
    ifstream trav(params.traversability_filename.c_str());
    if( ! trav){
      cerr << "ERROR: invalid traversability file \""
	   << params.traversability_filename << "\".\n";
      exit(EXIT_FAILURE);
    }
    traversability = parse_traversability(trav, cerr);
    if( ! traversability){
      cerr << "ERROR: parsing traversability file \""
	   << params.traversability_filename << "\".\n";
      exit(EXIT_FAILURE);
    }
    world.reset(new World(traversability->worldname));
    apply_traversability(traversability, world);
  }
  else{
    cerr << "ERROR: Specify either a world or a traversability file.\n";
    exit(EXIT_FAILURE);
  }
  
  simulator.
    reset(new Simulator(world, 0.000001 * timestep_usec, SimulatorMutex()));
  simulator->InitRobots(params.robot_config_filename);
  simulator->InitLayout(params.layout_config_filename, params.fatal_warnings);
  simulator->Init();
  
//   if(params.threaded){
//     update_thread.reset(new SimulatorUpdateThread("nepumuk", simulator));
//     update_thread->Start(timestep_usec);
//   }
  
  if(params.no_glut){
    simulator->SetContinuous();
    while(true){
      simulator->Idle();
      usleep(timestep_usec);
    }
  }
  else{
    init_glut(argc, argv, 700, 500);
    glutMainLoop();
  }
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
  ilock.Add(new StringCB(params.traversability_filename,
			 'm', "travmap", "Traversability map file."));
//   ilock.
//     Add(new Interlock::BoolCallback(params.threaded, 't', "threaded",
// 				    "Use separate GLUT thread."));
  ilock.
    Add(new Interlock::BoolCallback(params.no_glut, 'n', "no-glut",
				    "Disable graphic output."));
  ilock.
    Add(new Interlock::BoolCallback(params.fatal_warnings, 'f', "fwarn",
				    "Fatal warnings."));
  
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


void sighandle(int signum)
{
  cerr << "signal " << signum << ": exit\n";
  exit(EXIT_SUCCESS);
}


shared_ptr<travmap>
parse_traversability(istream & is, ostream & os)
{
  shared_ptr<travmap> result(new travmap());
  result->resolution = 0.1;
  result->origin_x = 0.0;
  result->origin_y = 0.0;
  result->origin_theta = 0.0;
  result->obstacle = 255;
  result->freespace = 0;
  result->worldname = "world";
  int thedefaultvalueman(0);
  
  vector<vector<int> > data;
  size_t grid_xsize(0);
  int maxdata(numeric_limits<int>::min());
  int mindata(numeric_limits<int>::max());
  
  string textline;
  while(getline(is, textline)){
    istringstream tls(textline);
    if(textline[0] == '#'){
      tls.ignore(1, '\n');
      string token;
      if(tls >> token){
	if(token == "resolution"){
	  tls >> result->resolution;
	  if( ! tls){
	    cerr << "ERROR: could not parse resolution from \""
		 << tls.str() << "\"\n";
	    return shared_ptr<travmap>();
	  }
	}
	else if(token == "origin"){
	  tls >> result->origin_x >> result->origin_y >> result->origin_theta;
	  if( ! tls){
	    cerr << "ERROR: could not parse origin from \""
		 << tls.str() << "\"\n";
	    return shared_ptr<travmap>();
	  }
	}
	else if(token == "obstacle"){
	  tls >> result->obstacle;
	  if( ! tls){
	    cerr << "ERROR: could not parse obstacle from \""
		 << tls.str() << "\"\n";
	    return shared_ptr<travmap>();
	  }
	}
	else if(token == "freespace"){
	  tls >> result->freespace;
	  if( ! tls){
	    cerr << "ERROR: could not parse freespace from \""
		 << tls.str() << "\"\n";
	    return shared_ptr<travmap>();
	  }
	}
	else if(token == "default"){
	  //	  tls >> result->default;
	  tls >> thedefaultvalueman;
	  if( ! tls){
	    cerr << "ERROR: could not parse default from \""
		 << tls.str() << "\"\n";
	    return shared_ptr<travmap>();
	  }
	}
	else if(token == "name"){
	  tls >> result->worldname;
	  if( ! tls){
	    cerr << "ERROR: could not parse name from \""
		 << tls.str() << "\"\n";
	    return shared_ptr<travmap>();
	  }
	  if(result->worldname.empty()){
	    cerr << "ERROR: empty names not allowed (from \""
		 << tls.str() << "\")\n";
	    return shared_ptr<travmap>();
	  }
	}
      }
      continue;
    }
    
    data.push_back(vector<int>());
    vector<int> & line(*data.rbegin());
    int value;
    while(tls >> value){
      line.push_back(value);
      if(value > maxdata)
	maxdata = value;
      if(value < mindata)
	mindata = value;
    }
    if(line.size() > grid_xsize)
      grid_xsize = line.size();
  }
  const size_t grid_ysize(data.size());
  
  if(grid_xsize < 1){
    os << "ERROR in parse_traversability(): grid_xsize == " << grid_xsize
       << "\n";
    return shared_ptr<travmap>();
  }
  if(grid_ysize < 1){
    os << "ERROR in parse_traversability(): grid_ysize == " << grid_ysize
       << "\n";
    return shared_ptr<travmap>();
  }
  
  //result->data.reset(new array2d<int>(grid_xsize, grid_ysize, result->default));
  result->data.reset(new array2d<int>(grid_xsize, grid_ysize, thedefaultvalueman));
  for(size_t iy(0); iy < grid_ysize; ++iy){
    vector<int> & line(data[iy]);
    for(size_t ix(0); ix < line.size(); ++ix)
      (*result->data)[ix][iy] = line[ix];
  }
  
  return result;
}


void add_line_n(size_t ix, size_t iy, const GridFrame & gframe,
		shared_ptr<World> world)
{
  const double offset(gframe.GetDelta() / 2);
  GridFrame::position_t center(gframe.LocalPoint(ix, iy));
  Line line(center.v0 - offset, center.v1 + offset,
	    center.v0 + offset, center.v1 + offset);
  line.TransformTo(gframe);
  world->AddLine(line);
}


void add_line_e(size_t ix, size_t iy, const GridFrame & gframe,
		shared_ptr<World> world)
{
  const double offset(gframe.GetDelta() / 2);
  GridFrame::position_t center(gframe.LocalPoint(ix, iy));
  Line line(center.v0 + offset, center.v1 - offset,
	    center.v0 + offset, center.v1 + offset);
  line.TransformTo(gframe);
  world->AddLine(line);
}


void add_line_s(size_t ix, size_t iy, const GridFrame & gframe,
		shared_ptr<World> world)
{
  const double offset(gframe.GetDelta() / 2);
  GridFrame::position_t center(gframe.LocalPoint(ix, iy));
  Line line(center.v0 - offset, center.v1 - offset,
	    center.v0 + offset, center.v1 - offset);
  line.TransformTo(gframe);
  world->AddLine(line);
}


void add_line_w(size_t ix, size_t iy, const GridFrame & gframe,
		shared_ptr<World> world)
{
  const double offset(gframe.GetDelta() / 2);
  GridFrame::position_t center(gframe.LocalPoint(ix, iy));
  Line line(center.v0 - offset, center.v1 - offset,
	    center.v0 - offset, center.v1 + offset);
  line.TransformTo(gframe);
  world->AddLine(line);
}


void apply_traversability(shared_ptr<const travmap> travmap,
			  shared_ptr<World> world)
{
  const GridFrame gframe(travmap->origin_x, travmap->origin_y,
			 travmap->origin_theta, travmap->resolution);
  for(size_t ix(0); ix < travmap->data->xsize; ++ix)
    for(size_t iy(0); iy < travmap->data->ysize; ++iy)
      if((*travmap->data)[ix][iy] >= travmap->obstacle){
	if((0 >= ix)
	   || ((*travmap->data)[ix - 1][iy] < travmap->obstacle))
	  add_line_w(ix, iy, gframe, world);
	if((travmap->data->xsize - 1 <= ix)
	   || ((*travmap->data)[ix + 1][iy] < travmap->obstacle))
	  add_line_e(ix, iy, gframe, world);
	if((0 >= iy)
	   || ((*travmap->data)[ix][iy - 1] < travmap->obstacle))
	  add_line_s(ix, iy, gframe, world);
	if((travmap->data->ysize - 1 <= iy)
	   || ((*travmap->data)[ix][iy + 1] < travmap->obstacle))
	  add_line_n(ix, iy, gframe, world);
      }
}
