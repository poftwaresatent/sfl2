/*
 * Copyright (c) 2005 CNRS/LAAS
 *
 * Author: Roland Philippsen <roland.philippsen@gmx.net>
 *
 * Permission to use, copy, modify, and distribute this software for any
 * purpose with or without fee is hereby granted, provided that the above
 * copyright notice and this permission notice appear in all copies.
 *
 * THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
 * WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
 * ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
 * WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
 * ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
 * OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
 */


#include "expo.h"
#include "sfl.h"
#include "Handlemap.hpp"
#include <sfl/api/HAL.hpp>
#include <sfl/api/Scanner.hpp>
#include <sfl/api/DiffDrive.hpp>
#include <sfl/api/Multiscanner.hpp>
#include <sfl/api/RobotModel.hpp>
#include <sfl/api/Odometry.hpp>
#include <sfl/dwa/DynamicWindow.hpp>
#include <sfl/bband/BubbleBand.hpp>
#include <sfl/expo/MotionPlanner.hpp>
#include <sfl/expo/MotionController.hpp>
#include <boost/shared_ptr.hpp>


using sfl::HAL;
using sfl::Hull;
using sfl::Polygon;
using sfl::Frame;
using sfl::Scanner;
using sfl::DiffDrive;
using sfl::Multiscanner;
using sfl::RobotModel;
using sfl::Odometry;
using sfl::DynamicWindow;
using sfl::BubbleBand;
using sfl::Goal;
using expo::MotionPlanner;
using expo::MotionController;
using boost::shared_ptr;


namespace sfl_cwrap {
  
  
  static Handlemap<MotionController> MotionController_map;
  static Handlemap<MotionPlanner>    MotionPlanner_map;
  
  
  shared_ptr<MotionController> get_MotionController(int handle)
  { return MotionController_map.Find(handle); }
  
  
  shared_ptr<MotionPlanner> get_MotionPlanner(int handle)
  { return MotionPlanner_map.Find(handle); }
  
}


using namespace sfl_cwrap;

  
int expo_create_MotionController(int RobotModel_handle,
				 int DiffDrive_handle)
{
  shared_ptr<RobotModel> rm(get_RobotModel(RobotModel_handle));
  if( ! rm)
    return -1;
  shared_ptr<DiffDrive> dd(get_DiffDrive(DiffDrive_handle));
  if( ! dd)
    return -2;
  return
    MotionController_map.InsertRaw(new MotionController("cwrap", *rm, *dd));
}
  
  
int expo_create_MotionPlanner(int MotionController_handle,
			      int DynamicWindow_handle,
			      int Multiscanner_handle,
			      int RobotModel_handle,
			      int BubbleBand_handle,
			      int Odometry_handle)
{
  shared_ptr<MotionController>
    mc(get_MotionController(MotionController_handle));
  if( ! mc)
    return -1;
  shared_ptr<DynamicWindow>
    dw(get_DynamicWindow(DynamicWindow_handle));
  if( ! dw)
    return -2;
  shared_ptr<Multiscanner>
    ms(get_Multiscanner(Multiscanner_handle));
  if( ! ms)
    return -3;
  shared_ptr<RobotModel>
    rm(get_RobotModel(RobotModel_handle));
  if( ! rm)
    return -4;
  shared_ptr<BubbleBand>
    bb(get_BubbleBand(BubbleBand_handle));
  if( ! bb)
    return -5;
  shared_ptr<Odometry>
    od(get_Odometry(Odometry_handle));
  if( ! od)
    return -6;
  return MotionPlanner_map.InsertRaw(new MotionPlanner(*mc, *dw, *ms,
						       *rm, *bb, *od));
}
  
  
int expo_factory(struct cwrap_hal_s * hal,
		 int front_sick_channel,
		 double front_sick_x,
		 double front_sick_y,
		 double front_sick_theta,
		 int rear_sick_channel,
		 double rear_sick_x,
		 double rear_sick_y,
		 double rear_sick_theta,		   
		 double wheelbase,
		 double wheelradius,
		 double timestep,
		 double security_distance,
		 double qd_max,
		 double qdd_max,
		 double sd_max,
		 double thetad_max,
		 double * hull_x,
		 double * hull_y,
		 int hull_len,
		 int dwa_dimension,
		 double dwa_grid_width,
		 double dwa_grid_height,
		 double dwa_grid_resolution,
		 double dwa_alpha_distance,
		 double dwa_alpha_heading,
		 double dwa_alpha_speed,
		 double bb_shortpath,
		 double bb_longpath,
		 double bb_max_ignore_distance)
{
  // TODO: avoid resource leak when returning errors
    
  int hal_handle(sfl_create_HAL(hal));
  if(0 > hal_handle)
    return -1;
    
  static const int sick_nscans(361);
  static const double sick_rhomax(8);
  static const double sick_phi0(-M_PI/2);
  static const double sick_phirange(M_PI);
    
  int sick_handle[2];
  sick_handle[0] = (sfl_create_Scanner(hal_handle, front_sick_channel,
				       "front_sick",
				       front_sick_x,
				       front_sick_y,
				       front_sick_theta,
				       sick_nscans,
				       sick_rhomax,
				       sick_phi0,
				       sick_phirange));
  if(0 > sick_handle[0])
    return -2;
  sick_handle[1] = (sfl_create_Scanner(hal_handle, rear_sick_channel,
				       "rear_sick",
				       rear_sick_x,
				       rear_sick_y,
				       rear_sick_theta,
				       sick_nscans,
				       sick_rhomax,
				       sick_phi0,
				       sick_phirange));
  if(0 > sick_handle[1])
    return -3;
  
  int drive_handle(sfl_create_DiffDrive(hal_handle, wheelbase, wheelradius));
  if(0 > drive_handle)
    return -4;
    
    
  const double sdd_max(0.75 * wheelradius * qdd_max);
  const double thetadd_max(1.5 * wheelradius * qdd_max / wheelbase);
    
  int model_handle(sfl_create_RobotModel(timestep, security_distance,
					 wheelbase, wheelradius,
					 qd_max, qdd_max,
					 sd_max, thetad_max,
					 sdd_max, thetadd_max,
					 hull_x, hull_y, hull_len));
  if(0 > model_handle)
    return -5;
    
  int mc_handle(expo_create_MotionController(model_handle, drive_handle));
  if(0 > mc_handle)
    return -6;
    
  int dwa_handle(sfl_create_DynamicWindow(model_handle,
					  mc_handle,
					  dwa_dimension,
					  dwa_grid_width,
					  dwa_grid_height,
					  dwa_grid_resolution,
					  dwa_alpha_distance,
					  dwa_alpha_heading,
					  dwa_alpha_speed));
  if(0 > dwa_handle)
    return -7;
    
  int odo_handle(sfl_create_Odometry(hal_handle));
  if(0 > odo_handle)
    return -8;
  
  int bb_handle(sfl_create_BubbleBand(model_handle,
				      odo_handle,
				      bb_shortpath,
				      bb_longpath,
				      bb_max_ignore_distance));
  if(0 > bb_handle)
    return -9;
    
  int ms_handle(sfl_create_Multiscanner(sick_handle, 2));
  if(0 > bb_handle)
    return -9;
    
  int mp_handle(expo_create_MotionPlanner(mc_handle,
					  dwa_handle,
					  ms_handle,
					  model_handle,
					  bb_handle,
					  odo_handle));
  if(0 > mp_handle)
    return -10;
    
  return mp_handle;
}


int expo_set_goal(int handle,
		  double x,
		  double y,
		  double theta,
		  double dr,
		  double dtheta,
		  int viaGoal)
{
  shared_ptr<MotionPlanner> mp(get_MotionPlanner(handle));
  if( ! mp)
    return -1;
  mp->SetGoal(Goal(x, y, theta, dr, dtheta, viaGoal));
  return 0;
}


int expo_goal_reached(int handle)
{
  shared_ptr<MotionPlanner> mp(get_MotionPlanner(handle));
  if( ! mp)
    return -1;
  return mp->GoalReached() ? 1 : 0;
}


int expo_update_all(int handle)
{
  shared_ptr<MotionPlanner> mp(get_MotionPlanner(handle));
  if( ! mp)
    return -1;
  mp->UpdateAll();
  return 0;
}
  
  
void expo_destroy_MotionController(int handle)
{ MotionController_map.Erase(handle); }
  

void expo_destroy_MotionPlanner(int handle)
{ MotionPlanner_map.Erase(handle); }
