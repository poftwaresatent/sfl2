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
#include "expoparams.hpp"
#include "OCamera.hpp"
#include "ODrawing.hpp"
#include "DODrawing.hpp"
#include "DWDrawing.hpp"
#include "RHDrawing.hpp"
#include "BBDrawing.hpp"
#include "GridLayerCamera.hpp"
#include "GridLayerDrawing.hpp"
#include "../common/World.hpp"
#include "../common/Globals.hpp"
#include "../common/OdometryDrawing.hpp"
#include "../common/StillCamera.hpp"
#include "../common/HAL.hpp"
#include "../common/DiffDrive.hpp"
#include "../common/RobotDescriptor.hpp"
#include "../common/Lidar.hpp"
#include "../common/util.hpp"
#include "../common/Manager.hpp"
#include <sfl/util/Pthread.hpp>
#include <sfl/util/pdebug.hpp>
#include <sfl/api/Odometry.hpp>
#include <sfl/api/Scanner.hpp>
#include <sfl/api/Multiscanner.hpp>
#include <sfl/api/Pose.hpp>
#include <sfl/api/RobotModel.hpp>
#include <sfl/api/Goal.hpp>
#include <sfl/dwa/DynamicWindow.hpp>
#include <sfl/dwa/DistanceObjective.hpp>
#include <sfl/dwa/SpeedObjective.hpp>
#include <sfl/dwa/HeadingObjective.hpp>
#include <sfl/bband/BubbleBand.hpp>
#include <sfl/expo/MotionPlanner.hpp>
#include <sfl/expo/MotionPlannerState.hpp>
#include <sfl/expo/MotionController.hpp>
#include <iostream>
#include <sstream>


#define PDEBUG PDEBUG_OFF


using namespace npm;
using namespace sfl;
using namespace boost;
using namespace std;


static const double LIDAROFFSET = 0.15;
static const double WHEELBASE = 0.521;
static const double WHEELRADIUS = 0.088;


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


BaseRobox::
BaseRobox(expoparams const & params,
	  boost::shared_ptr<sfl::HAL> hal,
	  boost::shared_ptr<sfl::Multiscanner> _mscan)
  : hull(CreateHull()),
    mscan(_mscan)
{
  RobotModel::Parameters const
    modelParms(params.model_security_distance,
	       params.model_wheelbase,
	       params.model_wheelradius,
	       params.model_qd_max,
	       params.model_qdd_max,
	       params.model_sd_max,
	       params.model_thetad_max,
	       params.model_sdd_max,
	       params.model_thetadd_max);
  robotModel.reset(new RobotModel(modelParms, hull));
  motionController.
    reset(new expo::MotionController(robotModel, hal,
				     RWlock::Create("motor")));
  odometry.reset(new Odometry(hal, RWlock::Create("odometry")));
  bubbleBand.
    reset(new BubbleBand(*robotModel, *odometry, *mscan,
			 BubbleList::Parameters(params.bband_shortpath,
						params.bband_longpath,
						params.bband_maxignoredistance),
			 RWlock::Create("bband")));
  
  dynamicWindow.reset(new DynamicWindow(params.dwa_dimension,
					params.dwa_grid_width,
					params.dwa_grid_height,
					params.dwa_grid_resolution,
					robotModel,
					params.dwa_alpha_distance,
					params.dwa_alpha_heading,
					params.dwa_alpha_speed,
					true));  
  motionPlanner.reset(new expo::MotionPlanner(motionController,
					      dynamicWindow,
					      mscan,
					      robotModel,
					      bubbleBand,
					      odometry));
  if ((params.mp_dtheta_starthoming > 0) && (params.mp_dtheta_startaiming > 0)) {
    if ( ! motionPlanner->SetAimingThresholds(params.mp_dtheta_startaiming,
					      params.mp_dtheta_starthoming)) {
      cerr << "ERROR: motionPlanner->SetAimingThresholds("
	   << params.mp_dtheta_startaiming << ", " << params.mp_dtheta_starthoming << ") failed\n";
      exit(EXIT_FAILURE);
    }
  }
}


Robox::
Robox(shared_ptr<RobotDescriptor> descriptor, const World & world)
  : RobotClient(descriptor, world, 2, true),
    m_ngkl(new local::NGKeyListener())
{
  expoparams params(descriptor);
  
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
  
  m_base.reset(new BaseRobox(params, GetHAL(), mscan));
  
  for(HullIterator ih(*m_base->hull); ih.IsValid(); ih.Increment()){
    AddLine(Line(ih.GetX0(), ih.GetY0(), ih.GetX1(), ih.GetY1()));
    PDEBUG("line %05.2f %05.2f %05.2f %05.2f\n",
	   ih.GetX0(), ih.GetY0(), ih.GetX1(), ih.GetY1());
  }
  
  CreateGfxStuff(descriptor->name);
  
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
CreateGfxStuff(const string & name)
{
  AddDrawing(new MPDrawing(name + "_goaldrawing", *m_base->motionPlanner));
  AddDrawing(new DWDrawing(name + "_dwdrawing", *m_base->dynamicWindow));
  AddDrawing(new ODrawing(name + "_dodrawing",
			  m_base->dynamicWindow->GetDistanceObjective(),
			  m_base->dynamicWindow));
  AddDrawing(new ODrawing(name + "_hodrawing",
			  m_base->dynamicWindow->GetHeadingObjective(),
			  m_base->dynamicWindow));
  AddDrawing(new ODrawing(name + "_sodrawing",
			  m_base->dynamicWindow->GetSpeedObjective(),
			  m_base->dynamicWindow));
  AddDrawing(new RHDrawing(name + "_rhdrawing",
			   m_base->bubbleBand->GetReplanHandler(),
			   RHDrawing::AUTODETECT));
  AddDrawing(new BBDrawing(name + "_bbdrawing",
			   *m_base->bubbleBand,
			   BBDrawing::AUTODETECT));
  AddDrawing(new GridLayerDrawing(name + "_local_gldrawing",
				  m_base->bubbleBand->GetReplanHandler()->GetNF1(),
				  false));
  AddDrawing(new GridLayerDrawing(name + "_global_gldrawing",
				  m_base->bubbleBand->GetReplanHandler()->GetNF1(),
				  true));
  AddDrawing(new OdometryDrawing(name + "_odomdrawing",
				 *m_base->odometry,
				 m_base->robotModel->WheelBase() / 2));
  AddDrawing(new DODrawing(name + "_collisiondrawing",
			   m_base->dynamicWindow->GetDistanceObjective(),
			   m_base->dynamicWindow,
			   m_base->robotModel));
  
  AddCamera(new StillCamera(name + "_dwcamera",
			    0,
			    0,
			    m_base->dynamicWindow->Dimension(),
			    m_base->dynamicWindow->Dimension(),
			    Instance<UniqueManager<Camera> >()));
  AddCamera(new OCamera(name + "_ocamera", *m_base->dynamicWindow));
  AddCamera(new GridLayerCamera(name + "_local_glcamera",
				m_base->bubbleBand->GetReplanHandler()->GetNF1()));
  double a, b, c, d;
  m_base->dynamicWindow->GetDistanceObjective()->GetRange(a, b, c, d);
  AddCamera(new StillCamera(name + "_collisioncamera", a, b, c, d,
			    Instance<UniqueManager<Camera> >()));
}


shared_ptr<Hull> BaseRobox::
CreateHull()
{
  shared_ptr<Hull> hull(new Hull());
  static const double octoSmall = 0.178;
  static const double octoBig = 0.430;
  
  Polygon outline;		// temporary
  outline.AddPoint( octoBig  , octoSmall);
  outline.AddPoint( octoSmall, octoBig);
  outline.AddPoint(-octoSmall, octoBig);
  outline.AddPoint(-octoBig  , octoSmall);
  outline.AddPoint(-octoBig  ,-octoSmall);
  outline.AddPoint(-octoSmall,-octoBig);
  outline.AddPoint( octoSmall,-octoBig);
  outline.AddPoint( octoBig  ,-octoSmall);
  hull->AddPolygon(outline);
  
  return hull;
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
  return shared_ptr<const Goal>(new Goal(m_base->motionPlanner->GetGoal()));
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
  m_base->motionPlanner->SetGoal(timestep, goal);
}


bool Robox::
PrepareAction(double timestep)
{
  m_base->mscan->UpdateAll();
  m_base->motionPlanner->Update(timestep);
  m_base->motionController->Update(timestep);
  m_base->odometry->Update();
  return true;
}
