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
#include <npm/common/World.hpp>
#include <npm/common/RobotClient.hpp>
#include <npm/common/RobotServer.hpp>
#include <npm/common/Manager.hpp>
#include <npm/common/SimpleImage.hpp>
#include <npm/common/View.hpp>
#include <npm/common/RobotDescriptor.hpp>
#include <sfl/util/Frame.hpp>
#include <sfl/util/Pthread.hpp>
#include <fstream>
#include <iostream>
#include <sstream>
#include <iomanip>

#include <sfl/util/pdebug.hpp>
#define PDEBUG PDEBUG_ERR
#define PVDEBUG PDEBUG_OFF


using namespace npm;
using namespace sfl;
using namespace boost;
using namespace std;


Simulator::
Simulator(shared_ptr<World> world, double timestep, shared_ptr<Mutex> mutex)
  : m_world(world),
    m_step(false),
    m_continuous(false),
    m_printscreen(false),
    m_timestep(timestep),
    m_mutex(mutex)
{
}


Simulator::
~Simulator()
{
}


void Simulator::
InitRobots(const string & filename)
{
  typedef shared_ptr<RobotDescriptor> rdesc_t;
  vector<rdesc_t> rdesc;
  ifstream config(filename.c_str());
  string token;
  while(config >> token){
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
    cerr << "ERROR in Simulator::InitRobots(): unknown token \""
	 << token << "\"\n";
    exit(EXIT_FAILURE);
  }
  
  // This is a bit ugly... but well, history is full of quirks: Set
  // the goals of all applicable robots *after* creating all robots,
  // because some code depends on the presence of all other robots
  // inside SetGoal. Notably, estar/Esbot.cpp.
  for(vector<rdesc_t>::iterator ir(rdesc.begin()); ir != rdesc.end(); ++ir){
    shared_ptr<RobotClient>
      rob(RobotFactory::Create((*ir), *m_world));
    if( ! rob){
      cerr << "ERROR in Simulator::InitRobots():\n"
	   << "   unknown model \"" << (*ir)->model << "\"\n";
      exit(EXIT_FAILURE);
    }
    m_robot.push_back(rob);
    m_world->AddRobot(rob->m_server.get());
    shared_ptr<const Frame> pose((*ir)->GetInitialPose());
    rob->m_server->InitializeTruePose(*pose);
    rob->SetPose(pose->X(), pose->Y(), pose->Theta());
  }
  for(size_t ir(0); ir < m_robot.size(); ++ir){
    rdesc_t rd(m_robot[ir]->GetDescriptor());
    if(rd->HaveGoals())
      m_robot[ir]->SetGoal(m_timestep, *rd->GetCurrentGoal());
  }
}


void Simulator::
InitLayout(const string &filename, bool fatal_warnings)
{
  ifstream config(filename.c_str());
  string token;
  View * view(0);
  ostringstream warnings;
  
  while(config >> token){
    if(token[0] == '#'){
      config.ignore(numeric_limits<streamsize>::max(), '\n');
      continue;
    }
    if(token == "View"){
      string foo;
      config >> foo;
      view = new View(foo);
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
    if(fatal_warnings)
      cerr << "ERROR in Simulator::InitLayout():\n";
    else
      cerr << "WARNING in Simulator::InitLayout():\n";
    cerr << warnings.str();
    Instance<UniqueManager<Drawing> >()->PrintCatalog(cerr);
    Instance<UniqueManager<Camera> >()->PrintCatalog(cerr);
    if(fatal_warnings)
      exit(EXIT_FAILURE);
  }
}


void Simulator::
Init()
{
  UpdateAllSensors();
}


void Simulator::
Reshape(int width,
	int height)
{
  Mutex::sentry sentry(m_mutex);
  m_width = width;
  m_height = height;
  Instance<UniqueManager<View> >()->Walk(View::ReshapeWalker(width, height));
}


void Simulator::
Draw()
{
  Mutex::sentry sentry(m_mutex);
  Instance<UniqueManager<View> >()->Walk(View::DrawWalker());
}


bool Simulator::
Idle()
{
  PVDEBUG("Hello!\n");
  bool retval(false);
  m_mutex->Lock();
  if(m_step || m_continuous){
    if(m_step)
      m_step = false;
    UpdateRobots();
    retval = true;
  }
  if(m_printscreen)
    PrintScreen();
  m_mutex->Unlock();
  return retval;
}


void Simulator::
UpdateAllSensors()
{
  for(robot_t::iterator ir(m_robot.begin()); ir != m_robot.end(); ++ir)
    (*ir)->m_server->UpdateAllSensors();
}


void Simulator::
UpdateRobots()
{
  PVDEBUG("updating robots...\n");
  
  Mutex::sentry sentry(m_mutex);

  UpdateAllSensors();
  for(robot_t::iterator ir(m_robot.begin()); ir != m_robot.end(); ++ir)
    (*ir)->PrepareAction(m_timestep);
  for(robot_t::iterator ir(m_robot.begin()); ir != m_robot.end(); ++ir)
    (*ir)->m_server->SimulateAction(m_timestep);
  
  for(robot_t::iterator ir(m_robot.begin()); ir != m_robot.end(); ++ir){
    if( ! (*ir)->GoalReached())
      continue;
    RobotDescriptor & desc(*(*ir)->GetDescriptor());
    if( ! desc.HaveGoals())
      continue;
    desc.NextGoal();
    (*ir)->SetGoal(m_timestep, *desc.GetCurrentGoal());
  }
}


void Simulator::
Keyboard(unsigned char key,
	 int mousex,
	 int mousey)
{
  m_world->DispatchKey(key);
  switch(key){
  case ' ':
    m_mutex->Lock();
    m_step = true;
    m_continuous = false;
    m_printscreen = false;
    m_mutex->Unlock();
    break;
  case 'c':
    m_mutex->Lock();
    m_step = false;
    m_continuous = true;
    m_printscreen = false;
    m_mutex->Unlock();
    break;
  case 'p':
    m_mutex->Lock();
    if( ! m_printscreen)
      PrintScreen();
    m_mutex->Unlock();
    break;
  case 'P':
    m_mutex->Lock();
    m_step = false;
    m_continuous = true;
    m_printscreen = true;
    m_mutex->Unlock();
    break;    
  case 'q':
    cout << "\nthanks, see you!\n";
    exit(EXIT_SUCCESS);
  }
}


void Simulator::
SetContinuous(bool printscreen)
{
  m_mutex->Lock();
  m_step = false;
  m_continuous = true;
  m_printscreen = printscreen;
  m_mutex->Unlock();
}


void Simulator::
PrintScreen()
{
  static unsigned int count(0);
  ostringstream filename;
  filename << "anim/pnf"
	   << setw(6) << setfill('0') << count++
	   << ".png";
  
  SimpleImage image(m_width, m_height);
  image.read_framebuf(0, 0);
  image.write_png(filename.str());
}


SimulatorUpdateThread::
SimulatorUpdateThread(const string & name,
		      shared_ptr<Simulator> simulator)
  : SimpleThread(name), m_simulator(simulator), m_changed(true)
{
}


bool SimulatorUpdateThread::
Changed()
{
  return m_changed;
}


void SimulatorUpdateThread::
Step()
{
  m_changed = m_simulator->Idle();
}
