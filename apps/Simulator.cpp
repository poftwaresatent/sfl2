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
#include <npm/ext/RobotFactory.hpp>
#include <npm/World.hpp>
#include <npm/RobotClient.hpp>
#include <npm/RobotServer.hpp>
#include <npm/gfx/PNGImage.hpp>
#include <npm/gfx/View.hpp>
#include <npm/gfx/Drawing.hpp>
#include <npm/gfx/Camera.hpp>
#include <npm/pdebug.hpp>
#include <sfl/util/Frame.hpp>
#include <sfl/api/Pose.hpp>
#include <fstream>
#include <iostream>
#include <sstream>
#include <iomanip>
#include <err.h>
#include <limits>


using namespace sfl;
using namespace boost;
using namespace std;


namespace npm {


  AppWindow::
  AppWindow(std::string const & _name, std::string const & _layout_filename,
	    int width, int height, Simulator * simul)
    : name(_name),
      layout_filename(_layout_filename),
      m_simul(simul),
      m_width(width),
      m_height(height)
  {
  }
  
  
  Simulator::
  Simulator(shared_ptr<World> world, double timestep,
	    std::string const & layout_filename,
	    bool _fatal_warnings)
    : fatal_warnings(_fatal_warnings),
      m_world(world),
      m_step(false),
      m_continuous(false),
      m_printscreen(false),
      m_timestep(timestep)
  {
    m_appwin.push_back(shared_ptr<AppWindow>(new AppWindow("nepumuk", layout_filename,
							   640, 480, this)));
  }


  Simulator::
  ~Simulator()
  {
  }


  bool Simulator::
  Initialize()
  {
    for (size_t ic(0); ic < RobotClient::registry->size(); ++ic) {
      RobotClient *client(RobotClient::registry->at(ic));
      RobotServer *server(new RobotServer(client, *m_world));
      if ( !client->Initialize(*server)) {
	delete server;
	delete client;
	return false;
      }
      m_world->AddRobot(server);
      m_robot.push_back(robot_s(server, client));
      Frame const pose(client->m_initial_pose.x, client->m_initial_pose.y, client->m_initial_pose.theta);
      server->InitializePose(pose);
      client->SetPose(Pose(client->m_initial_pose.x, client->m_initial_pose.y, client->m_initial_pose.theta));
      if (client->m_goals.size() > 0) {
	client->SetGoal(m_timestep, client->m_goals[0]);
      }
      
      // This was a nice feature that might be worth resurrecting using fpplib
      // if ( ! layout_file.empty())
      // 	m_appwin.push_back(shared_ptr<AppWindow>(new AppWindow(rdesc[ii]->name, layout_file,
      // 							       640, 480, this)));
    }
    
    UpdateAllSensors();
    
    return true;
  }


  void AppWindow::
  InitLayout()
  {
    ifstream config(layout_filename.c_str());
    string token;
    View *view = 0;
    ostringstream warnings;
  
    while(config >> token){
      if(token[0] == '#'){
	config.ignore(numeric_limits<streamsize>::max(), '\n');
	continue;
      }
      if (token == "Layout") {
	string foo;
	config >> foo;
	if (foo.empty()) {
	  cerr << "ERROR in Simulator::InitLayout(): empty key for Layout \""
	       << token << "\"\n";
	  exit(EXIT_FAILURE);
	}
	unsigned char const key(foo[0]);
	m_active_layout.reset(new layout_t());
	if ( ! m_default_layout)
	  m_default_layout = m_active_layout;
	m_layout[key] = m_active_layout;
      }
      else if(token == "View"){
	if ( ! m_active_layout) {
	  m_active_layout.reset(new layout_t());
	  m_default_layout = m_active_layout;
	}
	string foo;
	config >> foo;
	view = new View(foo);
	m_active_layout->add(view->name, view);
	m_views.push_back(view);
      }
      else if(token == "Camera"){
	string foo;
	config >> foo;
	if( ! view)
	  warnings << "  No View for Camera \"" << foo << "\".\n";
	else if( ! view->SetCamera(foo)){
	  warnings << "  View \"" << view->name << "\": "
		   << "no Camera \"" << foo << "\".\n";
	}
      }
      else if(token == "Drawing"){
	string foo;
	config >> foo;
	if( ! view)
	  warnings << "  No View for Drawing \"" << foo << "\".\n";
	else if( ! view->AddDrawing(foo)){
	  warnings << "  View \"" << view->name << "\": "
		   << "no Drawing \"" << foo << "\".\n";
	}
      }
      else if(token == "Window"){
	double x0, y0, x1, y1;
	config >> x0 >> y0 >> x1 >> y1;
	if( ! view)
	  warnings << "  No View for Window \"" << x0 << ", " << y0 << ", "
		   << x1 << ", " << y1 << "\".\n";
	else
	  view->Configure(x0, y0, x1 - x0, y1 - y0);
      }
      else if(token == "Border"){
	int border;
	config >> border;
	if(border < 0){
	  warnings << "  Border \"" << border << "\" < 0 for View \""
		   << view->name << "\", force 0.\n";
	  border = 0;
	}
	if( ! view)
	  warnings << "  No View for Border \"" << border << "\".\n";
	else
	  view->SetBorder(border);
      }
      else if(token == "Anchor"){
	string anchor;
	config >> anchor;
	if( ! view)
	  warnings << "  No View for Anchor \"" << anchor << "\".\n";
	else {
	  if(anchor == "N")
	    view->SetAnchor(View::N);
	  else if(anchor == "NE")
	    view->SetAnchor(View::NE);
	  else if(anchor == "E")
	    view->SetAnchor(View::E);
	  else if(anchor == "SE")
	    view->SetAnchor(View::SE);
	  else if(anchor == "S")
	    view->SetAnchor(View::S);
	  else if(anchor == "SW")
	    view->SetAnchor(View::SW);
	  else if(anchor == "W")
	    view->SetAnchor(View::W);
	  else if(anchor == "NW")
	    view->SetAnchor(View::NW);
	  else if(anchor == "CENTER")
	    view->SetAnchor(View::CENTER);
	  else
	    warnings << "  Unknown anchor \"" << anchor << "\" for View \""
		     << view->name << "\".\n";
	}
      }
      else{
	cerr << "ERROR in Simulator::InitLayout(): unknown token \""
	     << token << "\"\n";
	exit(EXIT_FAILURE);
      }
    }
  
    if( ! warnings.str().empty()){
      if(m_simul->fatal_warnings)
	cerr << "ERROR in Simulator::InitLayout():\n";
      else
	cerr << "WARNING in Simulator::InitLayout():\n";
      cerr << warnings.str()
	   << "\n==================================================\n"
	   << "CAMERAS:\n\n";
      for (Camera::registry_t::map_t::const_iterator ic(Camera::registry->map_.begin());
	   ic != Camera::registry->map_.end(); ++ic)
	cerr << "  " << ic->first << ": " << ic->second->comment << "\n";
      cerr << "\n==================================================\n"
	   << "DRAWINGS:\n\n";
      for (Drawing::registry_t::map_t::const_iterator id(Drawing::registry->map_.begin());
	   id != Drawing::registry->map_.end(); ++id)
	cerr << "  " << id->first << ": " << id->second->comment << "\n";
      if(m_simul->fatal_warnings)
	exit(EXIT_FAILURE);
    }
    
    if ( ! m_default_layout) {
      cerr << "ERROR in Simulator::InitLayout(): no default layout\n"
	   << "  you must specify at least one View\n";
      exit(EXIT_FAILURE);
    }    
    m_active_layout = m_default_layout;
    
    // inform all views about our window size (in pixels)
    Reshape(m_width, m_height);
  }


  void AppWindow::
  Reshape(int width,
	  int height)
  {
    // Do NOT try to be smart and skip if width and height haven't
    // changed, because we do call Reshape() with unchanged sizes when
    // view get initialized.
    
    m_width = width;
    m_height = height;
    
    for (layout_map_t::iterator il(m_layout.begin());
	 il != m_layout.end(); ++il)
      for (size_t iv(0); iv < il->second->size(); ++iv)
	il->second->at(iv)->Reshape(width, height);
    
    // It's a bit inefficient to reshape m_default_layout possibly
    // twice, but in case there is only a default layout it never gets
    // put into the m_layout map and we still have to update it.
    for (size_t iv(0); iv < m_default_layout->size(); ++iv)
      m_default_layout->at(iv)->Reshape(width, height);
  }
  
  
  void AppWindow::
  Draw()
  {
    for (size_t iv(0); iv < m_active_layout->size(); ++iv)
      m_active_layout->at(iv)->Draw();
  }


  bool Simulator::
  Idle()
  {
    PVDEBUG("Hello!\n");
    bool retval(false);
    if(m_step || m_continuous){
      if(m_step)
	m_step = false;
      UpdateRobots();
      retval = true;
    }
   if(m_printscreen)
     for (size_t ii(0); ii < m_appwin.size(); ++ii)
       m_appwin[ii]->PrintScreen();
    return retval;
  }


  void Simulator::
  UpdateAllSensors()
  {
    for(robot_t::iterator ir(m_robot.begin()); ir != m_robot.end(); ++ir)
      ir->server->UpdateAllSensors();
  }


  void Simulator::
  UpdateRobots()
  {
    PVDEBUG("updating robots...\n");
  
    UpdateAllSensors();
    for(robot_t::iterator ir(m_robot.begin()); ir != m_robot.end(); ++ir){
      ir->runnable = ir->client->PrepareAction(m_timestep);
      if(( ! ir->runnable) && m_continuous){
	m_continuous = false;
	m_step = true;
      }
    }
    for(robot_t::iterator ir(m_robot.begin()); ir != m_robot.end(); ++ir)
      if(ir->runnable)
	ir->server->SimulateAction(m_timestep);
  
    for(robot_t::iterator ir(m_robot.begin()); ir != m_robot.end(); ++ir){
      if( ! ir->runnable)
	continue;
      if( ! ir->client->GoalReached())
	continue;
      if(ir->client->m_goals.size() <= 1)
	continue;
      ir->goalidx = (ir->goalidx + 1) % ir->client->m_goals.size();
      ir->client->SetGoal(m_timestep, ir->client->m_goals[ir->goalidx]);
    }
  }


  void AppWindow::
  Keyboard(unsigned char key,
	   int mousex,
	   int mousey)
  {
    m_simul->m_world->DispatchKey(key);
    switch(key){
    case ' ':
      m_simul->m_step = true;
      m_simul->m_continuous = false;
      m_simul->m_printscreen = false;
      break;
    case 'c':
      m_simul->m_step = false;
      m_simul->m_continuous = true;
      m_simul->m_printscreen = false;
      break;
    case 'p':
      if( ! m_simul->m_printscreen)
	PrintScreen();
      break;
    case 'P':
      m_simul->m_step = false;
      m_simul->m_continuous = true;
      m_simul->m_printscreen = true;
      break;    
    case 'q':
      cout << "\nbye bye!\n";
      exit(EXIT_SUCCESS);
    default:
      layout_map_t::const_iterator il(m_layout.find(key));
      if (il != m_layout.end()) {
	if ((m_active_layout == m_default_layout)
	    || (m_active_layout != il->second))
	  m_active_layout = il->second;
	else
	  m_active_layout = m_default_layout;
      }
    }
  }


  void Simulator::
  SetContinuous(bool printscreen)
  {
    m_step = false;
    m_continuous = true;
    m_printscreen = printscreen;
  }


  void AppWindow::
  PrintScreen()
  {
#ifndef NPM_HAVE_PNG
    std::cerr << __FILE__": " << __func__ << ": PNG not supported in this build\n";
#else
    static unsigned int count(0);
    ostringstream filename;
    filename << "anim/" << name
	     << setw(6) << setfill('0') << count++
	     << ".png";
  
    PNGImage image(m_width, m_height);
    image.read_framebuf(0, 0);
    image.write_png(filename.str());
#endif
  }

}
