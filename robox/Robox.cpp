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
#include <npm/common/World.hpp>
#include <npm/common/Globals.hpp>
#include <npm/common/OdometryDrawing.hpp>
#include <npm/common/StillCamera.hpp>
#include <npm/common/HAL.hpp>
#include <npm/common/DiffDrive.hpp>
#include <npm/common/RobotDescriptor.hpp>
#include <npm/common/Lidar.hpp>
#include <npm/common/util.hpp>
#include <sfl/util/Pthread.hpp>
#include <sfl/util/pdebug.hpp>
#include <sfl/api/Odometry.hpp>
#include <sfl/api/Scanner.hpp>
#include <sfl/api/Multiscanner.hpp>
#include <sfl/api/Pose.hpp>
#include <sfl/api/RobotModel.hpp>
#include <sfl/api/Goal.hpp>
#include <sfl/dwa/DynamicWindow.hpp>
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
  
}


static RobotModel::Parameters
CreateRobotParameters(expoparams & params)
{
  return RobotModel::
    Parameters(params.model_security_distance,
	       params.model_wheelbase,
	       params.model_wheelradius,
	       params.model_qd_max,
	       params.model_qdd_max,
	       params.model_sd_max,
	       params.model_thetad_max,
	       params.model_sdd_max,
	       params.model_thetadd_max);
}


Robox::
Robox(shared_ptr<RobotDescriptor> descriptor, const World & world)
  : RobotClient(descriptor, world, true), m_hull(CreateHull())
{
  for(HullIterator ih(*m_hull); ih.IsValid(); ih.Increment()){
    AddLine(Line(ih.GetX0(), ih.GetY0(), ih.GetX1(), ih.GetY1()));
    PDEBUG("line %05.2f %05.2f %05.2f %05.2f\n",
	   ih.GetX0(), ih.GetY0(), ih.GetX1(), ih.GetY1());
  }
  
  expoparams params(descriptor);
  
  m_front = DefineLidar(Frame(params.front_mount_x,
			      params.front_mount_y,
			      params.front_mount_theta),
			params.front_nscans,
			params.front_rhomax,
			params.front_phi0,
			params.front_phirange,
			params.front_channel)->GetScanner();
  m_rear = DefineLidar(Frame(params.rear_mount_x,
			     params.rear_mount_y,
			     params.rear_mount_theta),
		       params.rear_nscans,
		       params.rear_rhomax,
		       params.rear_phi0,
		       params.rear_phirange,
		       params.rear_channel)->GetScanner();

  m_drive = DefineDiffDrive(params.model_wheelbase, params.model_wheelradius);
  const RobotModel::Parameters modelParms(CreateRobotParameters(params));
  m_robotModel.reset(new RobotModel(modelParms, m_hull));
  m_motionController.
    reset(new expo::MotionController(m_robotModel, GetHAL(),
				     RWlock::Create("motor")));
  m_odometry.reset(new Odometry(GetHAL(), RWlock::Create("odometry")));
  m_multiscanner.reset(new Multiscanner(m_odometry));
  m_bubbleBand.
    reset(new BubbleBand(*m_robotModel, *m_odometry, *m_multiscanner,
			 BubbleList::Parameters(params.bband_shortpath,
						params.bband_longpath,
					params.bband_maxignoredistance),
			 RWlock::Create("bband")));
  
  m_dynamicWindow.reset(new DynamicWindow(params.dwa_dimension,
					  params.dwa_grid_width,
					  params.dwa_grid_height,
					  params.dwa_grid_resolution,
					  m_robotModel,
					  *m_motionController,
					  params.dwa_alpha_distance,
					  params.dwa_alpha_heading,
					  params.dwa_alpha_speed,
					  true));
  
  m_multiscanner->Add(m_front);
  m_multiscanner->Add(m_rear);
  
  m_motionPlanner.reset(new expo::MotionPlanner(m_motionController,
						m_dynamicWindow,
						m_multiscanner,
						m_robotModel,
						m_bubbleBand,
						m_odometry));
  
  double dtheta_starthoming;
  if( ! string_to(descriptor->GetOption("dtheta_starthoming"),
		  dtheta_starthoming))
    dtheta_starthoming = -1;
  double dtheta_startaiming;
  if( ! string_to(descriptor->GetOption("dtheta_startaiming"),
		  dtheta_startaiming))
    dtheta_startaiming = -1;
  if((dtheta_starthoming > 0) && (dtheta_startaiming > 0)){
    if( ! m_motionPlanner->SetAimingThresholds(dtheta_startaiming,
					       dtheta_starthoming)){
      cerr << "ERROR: m_motionPlanner->SetAimingThresholds("
	   << dtheta_startaiming << ", " << dtheta_starthoming << ") failed\n";
      exit(EXIT_FAILURE);
    }
  }
  else if((dtheta_starthoming > 0) || (dtheta_startaiming > 0))
    cerr << "WARNING: you only set one of dtheta_starthoming or\n"
	 << "         dtheta_startaiming but should set both...\n"
	 << "         IGNORING WHATEVER YOU SET.\n";
  
  CreateGfxStuff(descriptor->name);
  
  shared_ptr<KeyListener> listener(new local::MPKeyListener(m_motionPlanner));
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
  AddDrawing(new MPDrawing(name + "_goaldrawing", *m_motionPlanner));
  AddDrawing(new DWDrawing(name + "_dwdrawing", *m_dynamicWindow));
  AddDrawing(new ODrawing(name + "_dodrawing",
			  m_dynamicWindow->GetDistanceObjective(),
			  *m_dynamicWindow));
  AddDrawing(new ODrawing(name + "_hodrawing",
			  m_dynamicWindow->GetHeadingObjective(),
			  *m_dynamicWindow));
  AddDrawing(new ODrawing(name + "_sodrawing",
			  m_dynamicWindow->GetSpeedObjective(),
			  *m_dynamicWindow));
  AddDrawing(new RHDrawing(name + "_rhdrawing",
			   m_bubbleBand->GetReplanHandler(),
			   RHDrawing::AUTODETECT));
  AddDrawing(new BBDrawing(name + "_bbdrawing",
			   *m_bubbleBand,
			   BBDrawing::AUTODETECT));
  AddDrawing(new GridLayerDrawing(name + "_local_gldrawing",
				  m_bubbleBand->GetReplanHandler()->GetNF1(),
				  false));
  AddDrawing(new GridLayerDrawing(name + "_global_gldrawing",
				  m_bubbleBand->GetReplanHandler()->GetNF1(),
				  true));
  AddDrawing(new OdometryDrawing(name + "_odomdrawing",
				 *m_odometry,
				 m_robotModel->WheelBase() / 2));
  AddDrawing(new DODrawing(name + "_collisiondrawing",
			   m_dynamicWindow->GetDistanceObjective(),
			   *m_dynamicWindow,
			   *m_robotModel));
  
  AddCamera(new StillCamera(name + "_dwcamera",
			    0,
			    0,
			    m_dynamicWindow->Dimension(),
			    m_dynamicWindow->Dimension(),
			    true));
  AddCamera(new OCamera(name + "_ocamera", *m_dynamicWindow));
  AddCamera(new GridLayerCamera(name + "_local_glcamera",
				m_bubbleBand->GetReplanHandler()->GetNF1()));
  double a, b, c, d;
  m_dynamicWindow->GetDistanceObjective().GetRange(a, b, c, d);
  AddCamera(new StillCamera(name + "_collisioncamera", a, b, c, d, true));
}


shared_ptr<Hull> Robox::
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
  m_odometry->Init(Pose(x, y, theta));
}


void Robox::
SetPose(double x,
	double y,
	double theta)
{
  m_odometry->Set(Pose(x, y, theta));
}


void Robox::
GetPose(double & x,
	double & y,
	double & theta)
{
  shared_ptr<const Pose> pose(m_odometry->Get());
  x = pose->X();
  y = pose->Y();
  theta = pose->Theta();
}


shared_ptr<const Goal> Robox::
GetGoal()
{
  return shared_ptr<const Goal>(new Goal(m_motionPlanner->GetGoal()));
}


bool Robox::
StartThreads()
{
  shared_ptr<RobotDescriptor> descriptor(GetDescriptor());
  if(descriptor->GetOption("front_thread") != ""){
    cerr << "front_thread \"" << descriptor->GetOption("front_thread")
	 << "\"\n";
    istringstream is(descriptor->GetOption("front_thread"));
    unsigned int useconds;
    if( ! (is >> useconds))
      return false;
    shared_ptr<ScannerThread> thread(new ScannerThread("front"));
    if( ! m_front->SetThread(thread))
      return false;
    if( ! thread->Start(useconds))
      return false;
    cerr << "started front_thread: " << useconds << "us\n";
  }
  if(descriptor->GetOption("rear_thread") != ""){
    cerr << "rear_thread \"" << descriptor->GetOption("rear_thread")
	 << "\"\n";
    istringstream is(descriptor->GetOption("rear_thread"));
    unsigned int useconds;
    if( ! (is >> useconds))
      return false;
    shared_ptr<ScannerThread> thread(new ScannerThread("rear"));
    if( ! m_rear->SetThread(thread))
      return false;
    if( ! thread->Start(useconds))
      return false;
    cerr << "started rear_thread: " << useconds << "us\n";
  }
  if(descriptor->GetOption("mc_thread") != ""){
    cerr << "mc_thread \"" << descriptor->GetOption("mc_thread")
	 << "\"\n";
    istringstream is(descriptor->GetOption("mc_thread"));
    unsigned int useconds;
    if( ! (is >> useconds))
      return false;
    shared_ptr<MotionControllerThread>
      thread(new MotionControllerThread("mc"));
    if( ! m_motionController->SetThread(thread))
      return false;
    if( ! thread->Start(useconds))
      return false;
    cerr << "started mc_thread: " << useconds << "us\n";
  }
  if(descriptor->GetOption("odo_thread") != ""){
    cerr << "odo_thread \"" << descriptor->GetOption("odo_thread")
	 << "\"\n";
    istringstream is(descriptor->GetOption("odo_thread"));
    unsigned int useconds;
    if( ! (is >> useconds))
      return false;
    shared_ptr<OdometryThread> thread(new OdometryThread("odo"));
    if( ! m_odometry->SetThread(thread))
      return false;
    if( ! thread->Start(useconds))
      return false;
    cerr << "started odo_thread: " << useconds << "us\n";
  }
  if(descriptor->GetOption("bband_thread") != ""){
    cerr << "bband_thread \"" << descriptor->GetOption("bband_thread")
	 << "\"\n";
    istringstream is(descriptor->GetOption("bband_thread"));
    unsigned int useconds;
    if( ! (is >> useconds))
      return false;
    shared_ptr<BubbleBandThread> thread(new BubbleBandThread("bband"));
    if( ! m_bubbleBand->SetThread(thread))
      return false;
    if( ! thread->Start(useconds))
      return false;
    cerr << "started bband_thread: " << useconds << "us\n";
  }
  return true;  
}


bool Robox::
GoalReached()
{
  return m_motionPlanner->GoalReached();
}


void Robox::
SetGoal(double timestep, const Goal & goal)
{
  m_motionPlanner->SetGoal(timestep, goal);
}


bool Robox::
PrepareAction(double timestep)
{
  m_front->Update();
  m_rear->Update();
  m_motionPlanner->Update(timestep);
  m_motionController->Update(timestep);
  m_odometry->Update();
  return true;
}
