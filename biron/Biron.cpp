/* 
 * Copyright (C) 2006 Roland Philippsen <roland dot philippsen at gmx dot net>
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


#include "Biron.hpp"
#include <npm/common/Lidar.hpp>
#include <npm/common/DiffDrive.hpp>
#include <npm/common/RobotDescriptor.hpp>
#include <npm/common/RobotServer.hpp>
#include <npm/common/HAL.hpp>
#include <sfl/util/pdebug.hpp>
#include <sfl/util/Line.hpp>
#include <sfl/api/Goal.hpp>
#include <sfl/api/Scanner.hpp>
#include <sfl/api/RobotModel.hpp>
#include <iostream>
#include <sstream>

#include <xcfwrap/xcfglue.h>


#define PDEBUG PDEBUG_ERR
#define PVDEBUG PDEBUG_ERR


using namespace npm;
using namespace sfl;
using namespace boost;
using namespace std;


static const double sick_x          = 0;
static const double sick_y          = 0;
static const double sick_theta      = 0;
static const int sick_channel       = 0;
static const size_t nscans          = 361;
static const double rhomax          = 8;
static const double phi0            = -M_PI/2;
static const double phirange        = M_PI;
static const double wheelbase       = 0.365;
static const double wheelradius     = 0.095;


static void init_xcfglue(shared_ptr<RobotDescriptor> descriptor)
{
  string name;
  int status;
  
  name = descriptor->GetOption("goalPublisher");
  if("" == name)
    name = "NavgoalData";
  PDEBUG("goal: %s\n", name.c_str());
  status = xcfglue_goal_publish(name.c_str());
  if(0 != status){
    cerr << "xcfglue_goal_publish() failed: " << status << "\n";
    exit(EXIT_FAILURE);
  }
  
  name = descriptor->GetOption("odometryPublisher");
  if("" == name)
    name = "OdometryData";
  PDEBUG("odometry: %s\n", name.c_str());
  status = xcfglue_odometry_publish(name.c_str());
  if(0 != status){
    cerr << "xcfglue_odometry_publish() failed: " << status << "\n";
    exit(EXIT_FAILURE);
  }
  
  name = descriptor->GetOption("scanPublisher");
  if("" == name)
    name = "LaserData";
  PDEBUG("scan: %s\n", name.c_str());
  status = xcfglue_scan_publish(name.c_str());
  if(0 != status){
    cerr << "xcfglue_scan_publish() failed: " << status << "\n";
    exit(EXIT_FAILURE);
  }
  
  name = descriptor->GetOption("curspeedPublisher");
  if("" == name)
    name = "SpeedData";
  PDEBUG("curspeed: %s\n", name.c_str());
  status = xcfglue_curspeed_publish(name.c_str());
  if(0 != status){
    cerr << "xcfglue_curspeed_publish() failed: " << status << "\n";
    exit(EXIT_FAILURE);
  }
  
  name = descriptor->GetOption("speedrefPublisher");
  if("" == name)
    name = "NAV_RelPos_Publisher";
  PDEBUG("speedref: %s\n", name.c_str());
  status = xcfglue_speedref_subscribe(name.c_str());
  while(-2 == status){		// HACK! TODO! XXX! hardcoded magic retval
    PDEBUG("no publisher called %s, will sleep a bit and retry\n",
	   name.c_str());
    usleep(2000000);
    status = xcfglue_speedref_subscribe(name.c_str());
  }
  if(0 != status){
    cerr << "xcfglue_speedref_subscribe() failed: " << status << "\n";
    exit(EXIT_FAILURE);
  }
  
  // DON'T FORGET DTOR IF YOU UNCOMMENT THIS!
  //   name = descriptor->GetOption("positionPublisher");
  //   if("" == name)
  //     name = "SlamPos";
  //   PDEBUG("position: %s\n", name.c_str());
  //   status = xcfglue_position_subscribe(name.c_str());
  //   if(0 != status){
  //     cerr << "xcfglue_position_subscribe() failed: " << status << "\n";
  //     exit(EXIT_FAILURE);
  //   }
}


static void cleanup_xcfglue()
{
  xcfglue_speedref_endsubscribe();
  //  xcfglue_position_endsubscribe();
  xcfglue_goal_endpublish();
  xcfglue_odometry_endpublish();
  xcfglue_scan_endpublish();
  xcfglue_curspeed_endpublish();
}


Biron::
Biron(shared_ptr<RobotDescriptor> descriptor, const World & world)
  : RobotClient(descriptor, world, true),
    m_goal(new Goal()), m_goal_changed(false), m_xcfglue_initialized(false)
{
  m_sick = DefineLidar(Frame(sick_x, sick_y, sick_theta), nscans, rhomax,
		       phi0, phirange, sick_channel)->GetScanner();
  DefineDiffDrive(wheelbase, wheelradius);
  m_hal = GetHAL();
  
  static const double xa = 0.1;
  static const double xb = 0.23;
  static const double xc = 0.28;
  static const double ya = 0.195;
  static const double yb = 0.045;
  AddLine(Line( xb,  yb,  xa,  ya));
  AddLine(Line( xa,  ya, -xc,  ya));
  AddLine(Line(-xc,  ya, -xc, -ya));
  AddLine(Line(-xc, -ya,  xa, -ya));
  AddLine(Line( xa, -ya,  xb, -yb));
  AddLine(Line( xb, -yb,  xb,  yb));
}


Biron::
~Biron()
{
  if(m_xcfglue_initialized)
    cleanup_xcfglue();
}


void Biron::
InitPose(double x, double y, double theta)
{
  const int status(m_hal->odometry_set(x, y, theta, 1, 1, 1, 0, 0, 0));
  if(0 != status){
    cerr << "npm::HAL::odometry_set() failed: " << status << "\n";
    exit(EXIT_FAILURE);
  }
}


void Biron::
SetPose(double x, double y, double theta)
{
  const int status(m_hal->odometry_set(x, y, theta, 1, 1, 1, 0, 0, 0));
  if(0 != status){
    cerr << "npm::HAL::odometry_set() failed: " << status << "\n";
    exit(EXIT_FAILURE);
  }
}


void Biron::
GetPose(double &x, double &y, double &theta)
{
  struct ::timespec bar;
  double foo;
  if(0 != m_hal->odometry_get(&bar, &x, &y, &theta,
			      &foo, &foo, &foo, &foo, &foo, &foo)){
    cerr << "npm::HAL::odometry_get() failed\n";
    exit(EXIT_FAILURE);
  }
}


void Biron::
SetGoal(double timestep, const Goal & goal)
{
  m_goal_changed = true;
  *m_goal = goal;
}


shared_ptr<const Goal> Biron::
GetGoal()
{
  return m_goal;
}


bool Biron::
GoalReached()
{
  return m_goal->Reached(GetServer().GetTruePose(), true);
}


void Biron::
PrepareAction(double timestep)
{
  if( ! m_xcfglue_initialized){
    PDEBUG("calling init_xcfglue()\n");
    init_xcfglue(GetDescriptor());
    m_xcfglue_initialized = true;
  }
  
  if(m_goal_changed){
    m_goal_changed = false;
    PDEBUG("new goal %05.2f %05.2f %05.2f %05.2f %05.2f\n", m_goal->X(),
	   m_goal->Y(), m_goal->Theta(), m_goal->Dr(), m_goal->Dtheta());
    const int
      status(xcfglue_goal_send(m_goal->X(), m_goal->Y(), m_goal->Theta(),
			       m_goal->Dr(), m_goal->Dtheta(),
			       xcfglue_system_timestamp(), "NAV"));
    if(0 != status){
      cerr << "xcfglue_goal_send() failed: " << status << "\n";
      exit(EXIT_FAILURE);
    }
  }
  
  {
    const Frame & pose(GetServer().GetTruePose());
    SetPose(pose.X(), pose.Y(), pose.Theta());
    PVDEBUG("odometry %05.2f %05.2f %05.2f\n",
	    pose.X(), pose.Y(), pose.Theta());
    const int status(xcfglue_odometry_send(pose.X(), pose.Y(), pose.Theta(),
					   xcfglue_system_timestamp(), "NAV"));
    if(0 != status){
      cerr << "xcfglue_odometry_send() failed: " << status << "\n";
      exit(EXIT_FAILURE);
    }
  }

  {
    m_sick->Update();		// for plotting, actually
    double rho[nscans];
    struct ::timespec t0, t1;
    size_t npm_nscans(nscans);
    int status(m_hal->scan_get(sick_channel, rho, &npm_nscans, &t0, &t1));
    if(0 != status){
      cerr << "npm::HAL::scan_get() failed: " << status << "\n";
      exit(EXIT_FAILURE);
    }
    PVDEBUG("scan...\n");
    int xcf_nscans(npm_nscans);
    status =
      xcfglue_scan_send(rho, &xcf_nscans, xcfglue_system_timestamp(), "NAV");
    if(0 != status){
      cerr << "xcfglue_scan_send() failed: " << status << "\n";
      exit(EXIT_FAILURE);
    }
    if(static_cast<size_t>(xcf_nscans) != npm_nscans){
      cerr << "nscans mismatch (xcf: " << xcf_nscans << " but npm: "
	   << npm_nscans << ")\n";
      exit(EXIT_FAILURE);
    }
  }
  
  {
    double qdl, qdr;
    int status(m_hal->speed_get(&qdl, &qdr));
    if(0 != status){
      cerr << "npm::HAL::speed_get() failed: " << status << "\n";
      exit(EXIT_FAILURE);
    }
    double sd, thetad;
    RobotModel::Actuator2Global(qdl, qdr, wheelbase, wheelradius, sd, thetad);
    PVDEBUG("curspeed %05.2f %05.2f\n", sd, thetad);
    status =
      xcfglue_curspeed_send(sd, thetad, xcfglue_system_timestamp(), "NAV");
    if(0 != status){
      cerr << "xcfglue_curspeed_send() failed: " << status << "\n";
      exit(EXIT_FAILURE);
    }
  }
  
  {
    double sd, thetad;
    unsigned long long tstamp;
    const int status(xcfglue_speedref_receive(&sd, &thetad, &tstamp));
    if(0 != status){
      cerr << "xcfglue_speedref_receive() failed: " << status << "\n";
      exit(EXIT_FAILURE);
    }
    PVDEBUG("speedref %05.2f %05.2f\n", sd, thetad);
    double qdl, qdr;
    RobotModel::Global2Actuator(sd, thetad, wheelbase, wheelradius, qdl, qdr);
    m_hal->speed_set(qdl, qdr);    
  }
  
  //   {
  //     double x, y, theta;
  //     unsigned long long tstamp;
  //     const int status(xcfglue_position_receive(&x, &y, &theta, &tstamp));
  //     if(0 != status){
  //       cerr << "xcfglue_position_receive() failed: " << status << "\n";
  //       exit(EXIT_FAILURE);
  //     }
  //     const int status(m_hal->odometry_set(x, y, theta, 1, 1, 1, 0, 0, 0));
  //     if(0 != status){
  //       cerr << "npm::HAL::odometry_set() failed: " << status << "\n";
  //       exit(EXIT_FAILURE);
  //     }
  //   }
}
