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
#include <npm/robox/expoparams.hpp>
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


Biron::
Biron(shared_ptr<RobotDescriptor> descriptor, const World & world)
  : RobotClient(descriptor, world, true),
    m_goal(new Goal()), m_goal_changed(false),
    m_speedref_status(-42), m_position_status(-42), m_goal_status(-42),
    m_odometry_status(-42), m_scan_status(-42), m_curspeed_status(-42)
{
  expoparams params(descriptor);
  m_nscans = params.front_nscans;
  m_sick_channel = params.front_channel;
  m_wheelbase = params.model_wheelbase;
  m_wheelradius = params.model_wheelradius;

  m_sick = DefineLidar(Frame(params.front_mount_x,
			     params.front_mount_y,
			     params.front_mount_theta),
		       params.front_nscans,
		       params.front_rhomax,
		       params.front_phi0,
		       params.front_phirange,
		       params.front_channel)->GetScanner();

  DefineDiffDrive(params.model_wheelbase, params.model_wheelradius);

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
  LazyXCFCleanup();
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
  LazyXCFInit();
  
  if(m_goal_changed){
    m_goal_changed = false;
    PDEBUG("new goal %05.2f %05.2f %05.2f %05.2f %05.2f\n", m_goal->X(),
	   m_goal->Y(), m_goal->Theta(), m_goal->Dr(), m_goal->Dtheta());
    if(0 == m_goal_status){
      const int
	status(xcfglue_goal_send(m_goal->X(), m_goal->Y(), m_goal->Theta(),
				 m_goal->Dr(), m_goal->Dtheta(),
				 xcfglue_system_timestamp(), "NAV"));
      if(0 != status)
	PDEBUG("xcfglue_goal_send() failed: %d\n", status);
    }
    else
      PDEBUG("m_goal_status: %d\n", m_goal_status);
  }
  
  if(0 == m_odometry_status){
    const Frame & pose(GetServer().GetTruePose()); // HAX!
    PVDEBUG("odometry %05.2f %05.2f %05.2f\n",
	    pose.X(), pose.Y(), pose.Theta());
    const int
      status(xcfglue_odometry_send(pose.X(), pose.Y(), pose.Theta(),
				   xcfglue_system_timestamp(), "NAV"));
    if(0 != status)
      PDEBUG("xcfglue_odometry_send() failed: %d\n", status);
  }
  else
    PDEBUG("m_odometry_status: %d\n", m_odometry_status);
  
  m_sick->Update();		// for plotting, actually
  double rho[m_nscans];
  struct ::timespec t0, t1;
  size_t npm_nscans(m_nscans);
  int status(m_hal->scan_get(m_sick_channel, rho, &npm_nscans, &t0, &t1));
  if(0 != status){
    cerr << "npm::HAL::scan_get() failed: " << status << "\n";
    exit(EXIT_FAILURE);
  }
  if(static_cast<size_t>(m_nscans) != npm_nscans){
    cerr << "nscans mismatch (me: " << m_nscans << " but npm: "
	 << npm_nscans << ")\n";
    exit(EXIT_FAILURE);
  }
  if(0 == m_scan_status){
    PVDEBUG("scan...\n");
    int xcf_nscans(npm_nscans);
    status =
      xcfglue_scan_send(rho, &xcf_nscans, xcfglue_system_timestamp(), "NAV");
    if(0 != status)
      PDEBUG("xcfglue_scan_send() failed: %d\n", status);
    else if(static_cast<size_t>(xcf_nscans) != npm_nscans){
      cerr << "nscans mismatch (xcf: " << xcf_nscans << " but npm: "
	   << npm_nscans << ")\n";
      exit(EXIT_FAILURE);
    }
  }
  else
    PDEBUG("m_scan_status: %d\n", m_scan_status);
  
  if(0 == m_curspeed_status){
    double qdl, qdr;
    int status(m_hal->speed_get(&qdl, &qdr));
    if(0 != status){
      cerr << "npm::HAL::speed_get() failed: " << status << "\n";
      exit(EXIT_FAILURE);
    }
    double sd, thetad;
    RobotModel::Actuator2Global(qdl, qdr, m_wheelbase, m_wheelradius,
				sd, thetad);
    PVDEBUG("curspeed %05.2f %05.2f\n", sd, thetad);
    status =
      xcfglue_curspeed_send(sd, thetad, xcfglue_system_timestamp(), "NAV");
    if(0 != status)
      PDEBUG("xcfglue_curspeed_send() failed: %d\n", status);
  }
  else
    PDEBUG("m_curspeed_status: %d\n", m_curspeed_status);      
  
  if(0 == m_speedref_status){
    double sd, thetad;
    unsigned long long tstamp;
    const int status(xcfglue_speedref_receive(&sd, &thetad, &tstamp));
    if(0 != status){
      PDEBUG("xcfglue_speedref_receive() failed: %d\n", status);
      m_hal->speed_set(0, 0);
    }
    else{
      PVDEBUG("speedref %05.2f %05.2f\n", sd, thetad);
      double qdl, qdr;
      RobotModel::Global2Actuator(sd, thetad, m_wheelbase, m_wheelradius,
				  qdl, qdr);
      m_hal->speed_set(qdl, qdr);
    }
  }
  else{
    PVDEBUG("m_speedref_status: %d\n", m_speedref_status);
    m_hal->speed_set(0, 0);
  }
  
  if(0 == m_position_status){
    double x, y, theta;
    unsigned long long tstamp;
    int status(xcfglue_position_receive(&x, &y, &theta, &tstamp));
    if(0 != status){
      cerr << "xcfglue_position_receive() failed: " << status << "\n";
      exit(EXIT_FAILURE);
    }
    PDEBUG("got slam pos: %05.2f %05.2f %05.2f\n", x, y, theta);
// //     status = m_hal->odometry_set(x, y, theta, 1, 1, 1, 0, 0, 0);
// //     if(0 != status){
// //       cerr << "npm::HAL::odometry_set() failed: " << status << "\n";
// //       exit(EXIT_FAILURE);
// //     }
  }
  else
    PVDEBUG("m_position_status: %d\n", m_position_status);      
}


void Biron::
LazyXCFInit()
{
  if(0 != m_goal_status){
    string name;
    name = GetDescriptor()->GetOption("goalPublisher");
    if("" == name)
      name = "NavgoalData";
    m_goal_status = xcfglue_goal_publish(name.c_str());
    if(0 != m_goal_status)
      PDEBUG("xcfglue_goal_publish() on %s failed with %d\n",
	     name.c_str(), m_goal_status);
    else
      PDEBUG("goal: %s\n", name.c_str());
  }
  
  if(0 != m_odometry_status){
    string name;
    name = GetDescriptor()->GetOption("odometryPublisher");
    if("" == name)
      name = "OdometryData";
    m_odometry_status = xcfglue_odometry_publish(name.c_str());
    if(0 != m_odometry_status)
      PDEBUG("xcfglue_odometry_publish() on %s failed with %d\n",
	     name.c_str(), m_odometry_status);
    else
      PDEBUG("odometry: %s\n", name.c_str());
  }
  
  if(0 != m_scan_status){
    string name;
    name = GetDescriptor()->GetOption("scanPublisher");
    if("" == name)
      name = "LaserData";
    m_scan_status = xcfglue_scan_publish(name.c_str());
    if(0 != m_scan_status)
      PDEBUG("xcfglue_scan_publish() on %s failed with %d\n",
	     name.c_str(), m_scan_status);
    else
      PDEBUG("scan: %s\n", name.c_str());
  }
  
  if(0 != m_curspeed_status){
    string name;
    name = GetDescriptor()->GetOption("curspeedPublisher");
    if("" == name)
      name = "SpeedData";
    m_curspeed_status = xcfglue_curspeed_publish(name.c_str());
    if(0 != m_curspeed_status)
      PDEBUG("xcfglue_curspeed_publish() on %s failed with %d\n",
	     name.c_str(), m_curspeed_status);
    else
      PDEBUG("curspeed: %s\n", name.c_str());
  }
  
  if(0 != m_speedref_status){
    string name;
    name = GetDescriptor()->GetOption("speedrefPublisher");
    if("" == name)
      name = "NAV_RelPos_Publisher";
    m_speedref_status = xcfglue_speedref_subscribe(name.c_str());
    if(0 != m_speedref_status)
      PDEBUG("xcfglue_speedref_subscribe() on %s failed with %d\n",
	     name.c_str(), m_speedref_status);
    else
      PDEBUG("speedref: %s\n", name.c_str());
//     while(-2 == status){
//       PDEBUG("no publisher called %s, will sleep a bit and retry\n",
// 	     name.c_str());
//       usleep(2000000);
//       status = xcfglue_speedref_subscribe(name.c_str());
//     }
  }
  
  if(0 != m_position_status){
    string name;
    name = GetDescriptor()->GetOption("positionPublisher");
    if("" == name)
      name = "SlamPos";
    m_position_status = xcfglue_position_subscribe(name.c_str());
    if(0 != m_position_status)
      PDEBUG("xcfglue_position_subscribe() on %s failed with %d\n",
	     name.c_str(), m_position_status);
    else
      PDEBUG("position: %s\n", name.c_str());
  }
}


void Biron::
LazyXCFCleanup()
{
  if(0 == m_speedref_status) xcfglue_speedref_endsubscribe();
  if(0 == m_position_status) xcfglue_position_endsubscribe();
  if(0 == m_goal_status)     xcfglue_goal_endpublish();
  if(0 == m_odometry_status) xcfglue_odometry_endpublish();
  if(0 == m_scan_status)     xcfglue_scan_endpublish();
  if(0 == m_curspeed_status) xcfglue_curspeed_endpublish();
}
