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


#include "Robox.hpp"
#include "MPDrawing.hpp"
#include "OCamera.hpp"
#include "ODrawing.hpp"
#include "DODrawing.hpp"
#include "DWDrawing.hpp"
#include "RHDrawing.hpp"
#include "BBDrawing.hpp"
#include "GridLayerCamera.hpp"
#include "GridLayerDrawing.hpp"
#include "../common/World.hpp"
#include "../common/OdometryDrawing.hpp"
#include "../common/StillCamera.hpp"
#include "../common/HAL.hpp"
#include "../common/RobotDescriptor.hpp"
#include "../common/Lidar.hpp"
#include "../common/pdebug.hpp"
#include "../common/Manager.hpp"
#include <sfl/api/Odometry.hpp>
#include <sfl/api/Multiscanner.hpp>
#include <sfl/api/RobotModel.hpp>
#include <sfl/dwa/DynamicWindow.hpp>
#include <sfl/dwa/DistanceObjective.hpp>
#include <sfl/dwa/SpeedObjective.hpp>
#include <sfl/dwa/HeadingObjective.hpp>
#include <sfl/expo/expo_parameters.h>
#include <sfl/expo/MotionPlanner.hpp>
#include <sfl/expo/MotionController.hpp>
#include <sfl/expo/Robox.hpp>


using namespace npm;
using namespace sfl;
using namespace boost;
using namespace std;


namespace local {
  
  class MPKeyListener: public KeyListener {
  public:
    MPKeyListener(shared_ptr<expo::MotionPlanner> _mp)
      : mp(_mp), stopped(false) {}
    
    virtual void KeyPressed(unsigned char key)
    {
      if('m' != key)
	return;
      if(stopped){
	stopped = false;
	mp->ManualResume();
      }
      else{
	stopped = true;
	mp->ManualStop();
      }
    }
    
    shared_ptr<expo::MotionPlanner> mp;
    bool stopped;
  };
  
  
  class NGKeyListener: public KeyListener {
  public:
    NGKeyListener(): next_goal(false) {}
    
    virtual void KeyPressed(unsigned char key)
    { if('n' == key) next_goal = true; }
    
    bool next_goal;
  };
  
}


namespace npm {

  VisualRobox::
  VisualRobox(std::string const & name,
	      expo_parameters const & params,
	      boost::shared_ptr<sfl::HAL> hal,
	      boost::shared_ptr<sfl::Multiscanner> mscan)
    : expo::Robox(params, hal, mscan)
  {
    AddDrawing(new MPDrawing(name + "_goaldrawing", *motionPlanner));
    AddDrawing(new DWDrawing(name + "_dwdrawing", *dynamicWindow));
    AddDrawing(new ODrawing(name + "_dodrawing",
			    dynamicWindow->GetDistanceObjective(),
			    dynamicWindow));
    AddDrawing(new ODrawing(name + "_hodrawing",
			    dynamicWindow->GetHeadingObjective(),
			    dynamicWindow));
    AddDrawing(new ODrawing(name + "_sodrawing",
			    dynamicWindow->GetSpeedObjective(),
			    dynamicWindow));
    AddDrawing(new RHDrawing(name + "_rhdrawing",
			     bubbleBand->GetReplanHandler(),
			     RHDrawing::AUTODETECT));
    AddDrawing(new BBDrawing(name + "_bbdrawing",
			     *bubbleBand,
			     BBDrawing::AUTODETECT));
    AddDrawing(new GridLayerDrawing(name + "_local_gldrawing",
				    bubbleBand->GetReplanHandler()->GetNF1(),
				    false));
    AddDrawing(new GridLayerDrawing(name + "_global_gldrawing",
				    bubbleBand->GetReplanHandler()->GetNF1(),
				    true));
    AddDrawing(new OdometryDrawing(name + "_odomdrawing",
				   *odometry,
				   robotModel->WheelBase() / 2));
    AddDrawing(new DODrawing(name + "_collisiondrawing",
			     dynamicWindow->GetDistanceObjective(),
			     dynamicWindow,
			     robotModel));
    
    AddCamera(new StillCamera(name + "_dwcamera",
			      0,
			      0,
			      dynamicWindow->Dimension(),
			      dynamicWindow->Dimension(),
			      Instance<UniqueManager<Camera> >()));
    AddCamera(new OCamera(name + "_ocamera", *dynamicWindow));
    AddCamera(new GridLayerCamera(name + "_local_glcamera",
				  bubbleBand->GetReplanHandler()->GetNF1()));
    double a, b, c, d;
    dynamicWindow->GetDistanceObjective()->GetRange(a, b, c, d);
    AddCamera(new StillCamera(name + "_collisioncamera", a, b, c, d,
			      Instance<UniqueManager<Camera> >()));
  }
  
  
  void VisualRobox::
  AddDrawing(Drawing * drawing)
  {
    m_drawing.push_back(shared_ptr<Drawing>(drawing));
  }
  
  
  void VisualRobox::
  AddCamera(Camera * camera)
  {
    m_camera.push_back(shared_ptr<Camera>(camera));
  }
  
}


Robox::
Robox(shared_ptr<RobotDescriptor> descriptor, const World & world)
  : RobotClient(descriptor, world, 2, true),
    m_ngkl(new local::NGKeyListener())
{
  expo_parameters params(descriptor);
  
  boost::shared_ptr<sfl::Scanner>
    front = DefineLidar(Frame(params.front_mount_x,
			      params.front_mount_y,
			      params.front_mount_theta),
			params.front_nscans,
			params.front_rhomax,
			params.front_phi0,
			params.front_phirange,
			params.front_channel)->GetScanner();
  boost::shared_ptr<sfl::Scanner>
    rear = DefineLidar(Frame(params.rear_mount_x,
			     params.rear_mount_y,
			     params.rear_mount_theta),
		       params.rear_nscans,
		       params.rear_rhomax,
		       params.rear_phi0,
		       params.rear_phirange,
		       params.rear_channel)->GetScanner();
  boost::shared_ptr<sfl::Multiscanner> mscan(new Multiscanner(GetHAL()));
  mscan->Add(front);
  mscan->Add(rear);
  
  m_drive = DefineDiffDrive(params.model_wheelbase, params.model_wheelradius);
  
  m_base.reset(new npm::VisualRobox(descriptor->name, params, GetHAL(), mscan));
  
  for (HullIterator ih(*m_base->hull); ih.IsValid(); ih.Increment()) {
    AddLine(Line(ih.GetX0(), ih.GetY0(), ih.GetX1(), ih.GetY1()));
    PDEBUG("line %05.2f %05.2f %05.2f %05.2f\n",
	   ih.GetX0(), ih.GetY0(), ih.GetX1(), ih.GetY1());
  }
  
  world.AddKeyListener(m_ngkl);
  shared_ptr<KeyListener> listener(new local::MPKeyListener(m_base->motionPlanner));
  world.AddKeyListener(listener);
}


Robox * Robox::
Create(shared_ptr<RobotDescriptor> descriptor, const World & world)
{
  Robox * robox(new Robox(descriptor, world));
  if( ! robox->StartThreads()){
    delete robox;
    return 0;
  }
  return robox;
}


void Robox::
InitPose(double x,
	 double y,
	 double theta)
{
  m_base->odometry->Init(Pose(x, y, theta));
}


void Robox::
SetPose(double x,
	double y,
	double theta)
{
  m_base->odometry->Set(Pose(x, y, theta));
}


void Robox::
GetPose(double & x,
	double & y,
	double & theta)
{
  shared_ptr<const Pose> pose(m_base->odometry->Get());
  x = pose->X();
  y = pose->Y();
  theta = pose->Theta();
}


shared_ptr<const Goal> Robox::
GetGoal()
{
  return shared_ptr<const Goal>(new Goal(m_base->GetGoal()));
}


bool Robox::
StartThreads()
{
  shared_ptr<RobotDescriptor> descriptor(GetDescriptor());
  if ((descriptor->GetOption("front_thread") != "")
      || (descriptor->GetOption("rear_thread") != "")
      || (descriptor->GetOption("mc_thread") != "")
      || (descriptor->GetOption("odo_thread") != "")
      || (descriptor->GetOption("bband_thread") != ""))
    cerr << "Robox::StartThreads(): threading was never really working, so it was kicked out.\n"
	 << "  the options front_thread, rear_thread, mc_thread, odo_thread, and bband_thread\n"
	 << "  are simply being ignored. Have  anice day.\n";
  return true;  
}


bool Robox::
GoalReached()
{
  if(m_ngkl->next_goal){
    m_ngkl->next_goal = false;
    return true;
  }
  return m_base->motionPlanner->GoalReached();
}


void Robox::
SetGoal(double timestep, const Goal & goal)
{
  m_base->SetGoal(timestep, goal);
}


bool Robox::
PrepareAction(double timestep)
{
  try {
    m_base->Update(timestep);
  }
  catch (runtime_error ee) {
    cerr << "Robox::PrepareAction(): m_base->Update() failed with exception\n"
	 << ee.what() << "\n";
    return false;
  }
  return true;
}
