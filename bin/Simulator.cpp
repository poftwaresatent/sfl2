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
#include "RobotFactory.hpp"
#include "../common/World.hpp"
#include "../common/RobotClient.hpp"
#include "../common/RobotServer.hpp"
#include "../common/Manager.hpp"
#include "../common/SimpleImage.hpp"
#include "../common/View.hpp"
#include "../common/RobotDescriptor.hpp"
#include "../common/pdebug.hpp"
#include <sfl/util/Frame.hpp>
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
	    std::string const & _robot_config_filename,
	    std::string const & layout_filename,
	    bool _fatal_warnings)
    : robot_config_filename(_robot_config_filename),
      fatal_warnings(_fatal_warnings),
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


  void Simulator::
  InitRobots()
  {
    typedef shared_ptr<RobotDescriptor> rdesc_t;
    vector<rdesc_t> rdesc;
    ifstream config(robot_config_filename.c_str());
    string token;
    for (int linenumber(1); config >> token; ++linenumber) {
      if(token[0] == '#'){
	config.ignore(numeric_limits<streamsize>::max(), '\n');
	continue;
      }
      if(token == "Robot"){
	string model;
	if( ! (config >> model)){
	  cerr << "ERROR in Simulator::InitRobots():\n"
	       << "  can't read robot model\n";
	  exit(EXIT_FAILURE);
	}
	string name;
	if( ! (config >> name)){
	  cerr << "ERROR in Simulator::InitRobots():\n"
	       << "  can't read robot name\n";
	  exit(EXIT_FAILURE);
	}
	rdesc.push_back(rdesc_t(new RobotDescriptor(model, name)));
	continue;
      }
      if(token == "Pose"){
	double x;
	if( ! (config >> x)){
	  cerr << "ERROR in Simulator::InitRobots():\n"
	       << "  can't read initial x\n";
	  exit(EXIT_FAILURE);
	}
	double y;
	if( ! (config >> y)){
	  cerr << "ERROR in Simulator::InitRobots():\n"
	       << "  can't read initial y\n";
	  exit(EXIT_FAILURE);
	}
	double theta;
	if( ! (config >> theta)){
	  cerr << "ERROR in Simulator::InitRobots():\n"
	       << "  can't read initial theta\n";
	  exit(EXIT_FAILURE);
	}
	rdesc.back()->SetInitialPose(x, y, theta);
	continue;
      }
      if(token == "Goal"){
	//      double x, y, theta, dr, dtheta;
	double x;
	if( ! (config >> x)){
	  cerr << "ERROR in Simulator::InitRobots():\n"
	       << "  can't read goal x\n";
	  exit(EXIT_FAILURE);
	}
	double y;
	if( ! (config >> y)){
	  cerr << "ERROR in Simulator::InitRobots():\n"
	       << "  can't read goal y\n";
	  exit(EXIT_FAILURE);
	}
	double theta;
	if( ! (config >> theta)){
	  cerr << "ERROR in Simulator::InitRobots():\n"
	       << "  can't read goal theta\n";
	  exit(EXIT_FAILURE);
	}
	double dr;
	if( ! (config >> dr)){
	  cerr << "ERROR in Simulator::InitRobots():\n"
	       << "  can't read goal dr\n";
	  exit(EXIT_FAILURE);
	}
	double dtheta;
	if( ! (config >> dtheta)){
	  cerr << "ERROR in Simulator::InitRobots():\n"
	       << "  can't read goal dtheta\n";
	  exit(EXIT_FAILURE);
	}
	rdesc.back()->AddGoal(x, y, theta, dr, dtheta);
	continue;
      }
      if(token == "EndGoal"){
	//      double x, y, theta, dr, dtheta;
	double x;
	if( ! (config >> x)){
	  cerr << "ERROR in Simulator::InitRobots():\n"
	       << "  can't read goal x\n";
	  exit(EXIT_FAILURE);
	}
	double y;
	if( ! (config >> y)){
	  cerr << "ERROR in Simulator::InitRobots():\n"
	       << "  can't read goal y\n";
	  exit(EXIT_FAILURE);
	}
	double theta;
	if( ! (config >> theta)){
	  cerr << "ERROR in Simulator::InitRobots():\n"
	       << "  can't read goal theta\n";
	  exit(EXIT_FAILURE);
	}
	double dr;
	if( ! (config >> dr)){
	  cerr << "ERROR in Simulator::InitRobots():\n"
	       << "  can't read goal dr\n";
	  exit(EXIT_FAILURE);
	}
	double dtheta;
	if( ! (config >> dtheta)){
	  cerr << "ERROR in Simulator::InitRobots():\n"
	       << "  can't read goal dtheta\n";
	  exit(EXIT_FAILURE);
	}
	rdesc.back()->AddEndGoal(x, y, theta, dr, dtheta);
	continue;
      }
      if(token == "Option"){
	string name;
	if( ! (config >> name)){
	  cerr << "ERROR in Simulator::InitRobots():\n"
	       << "  can't read option name\n";
	  exit(EXIT_FAILURE);
	}
	string value;
	if( ! (config >> value)){
	  cerr << "ERROR in Simulator::InitRobots():\n"
	       << "  can't read option value\n";
	  exit(EXIT_FAILURE);
	}
	rdesc.back()->SetOption(name, value);
	continue;
      }
      else {
	string rest;
	getline(config, rest);
	if ( ! config) {
	  cerr << "ERROR in Simulator::InitRobots():\n"
	       << "  cannot extract rest of line aftern token `" << token << "'\n";
	  exit(EXIT_FAILURE);
	}
	ostringstream os;
	os << token << " " << rest;
	rdesc.back()->AddCustomLine(linenumber, os.str());
	PDEBUG ("added custom line %d: %s\n", linenumber, os.str().c_str());
      }
    }
    
    // Historic quirk: set the goals of all applicable robots *after*
    // creating all robots, some code depends on the presence of all
    // other robots inside SetGoal.
    
    for (size_t ii(0); ii < rdesc.size(); ++ii) {
      
      // create robot from descriptor
      shared_ptr<RobotClient> rob(RobotFactory::Create(rdesc[ii], *m_world));
      if ( ! rob)
	errx(EXIT_FAILURE,
	     "Simulator::InitRobots(): unknown model \"%s\" or parse error in robot construction",
	     rdesc[ii]->model.c_str());

      // hook it into the simulation
      m_robot.push_back(robot_s(rob));
      m_world->AddRobot(rob->m_server.get());
      shared_ptr<const Frame> pose(rdesc[ii]->GetInitialPose());
      rob->m_server->InitializePose(*pose);
      rob->SetPose(pose->X(), pose->Y(), pose->Theta());
      
      // did this robot request a standalone window?
      string const layout_file(rdesc[ii]->GetOption("standalone_window"));
      if ( ! layout_file.empty())
	m_appwin.push_back(shared_ptr<AppWindow>(new AppWindow(rdesc[ii]->name, layout_file,
							       640, 480, this)));
    }
    
    for (size_t ir(0); ir < m_robot.size(); ++ir) {
      rdesc_t rd(m_robot[ir].robot->GetDescriptor());
      if(rd->HaveGoals())
	m_robot[ir].robot->SetGoal(m_timestep, *rd->GetCurrentGoal());
    }
  }


  void AppWindow::
  InitLayout()
  {
    ifstream config(layout_filename.c_str());
    string token;
    view_ptr view;
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
	view.reset(new View(foo, m_active_layout));
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
      Instance<UniqueManager<Camera> >()->PrintCatalog(cerr);
      cerr << "\n==================================================\n"
	   << "DRAWINGS:\n\n";
      Instance<UniqueManager<Drawing> >()->PrintCatalog(cerr);
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
  
  
  void Simulator::
  Init()
  {
    UpdateAllSensors();
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
    
    View::ReshapeWalker const walker(width, height);
    for (layout_map_t::iterator il(m_layout.begin());
	 il != m_layout.end(); ++il)
      il->second->Walk(walker);
    // It's a bit inefficient to reshape m_default_layout possibly
    // twice, but in case there is only a default layout it never gets
    // put into the m_layout map and we still have to update it.
    m_default_layout->Walk(walker);
  }
  
  
  void AppWindow::
  Draw()
  {
    m_active_layout->Walk(View::DrawWalker());
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
      ir->robot->m_server->UpdateAllSensors();
  }


  void Simulator::
  UpdateRobots()
  {
    PVDEBUG("updating robots...\n");
  
    UpdateAllSensors();
    for(robot_t::iterator ir(m_robot.begin()); ir != m_robot.end(); ++ir){
      ir->runnable = ir->robot->PrepareAction(m_timestep);
      if(( ! ir->runnable) && m_continuous){
	m_continuous = false;
	m_step = true;
      }
    }
    for(robot_t::iterator ir(m_robot.begin()); ir != m_robot.end(); ++ir)
      if(ir->runnable)
	ir->robot->m_server->SimulateAction(m_timestep);
  
    for(robot_t::iterator ir(m_robot.begin()); ir != m_robot.end(); ++ir){
      if( ! ir->runnable)
	continue;
      if( ! ir->robot->GoalReached())
	continue;
      RobotDescriptor & desc(*ir->robot->GetDescriptor());
      if( ! desc.HaveGoals())
	continue;
      desc.NextGoal();
      ir->robot->SetGoal(m_timestep, *desc.GetCurrentGoal());
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
    static unsigned int count(0);
    ostringstream filename;
    filename << "anim/" << name
	     << setw(6) << setfill('0') << count++
	     << ".png";
  
    SimpleImage image(m_width, m_height);
    image.read_framebuf(0, 0);
    image.write_png(filename.str());
  }

}
