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
#include <boost/scoped_ptr.hpp>
#include <boost/shared_ptr.hpp>
#include <map>


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
using expo::MotionPlanner;
using expo::MotionController;
using boost::scoped_ptr;
using boost::shared_ptr;
using std::map;
using std::make_pair;


class Handle {
public:
  shared_ptr<Scanner>          front_sick;
  shared_ptr<Scanner>          rear_sick;
  scoped_ptr<DiffDrive>        drive;
  scoped_ptr<RobotModel>       robot_model;
  scoped_ptr<MotionController> motion_controller;
  scoped_ptr<DynamicWindow>    dynamic_window;
  scoped_ptr<Odometry>         odometry;
  scoped_ptr<BubbleBand>       bubble_band;
  scoped_ptr<Multiscanner>     multiscanner;
  scoped_ptr<MotionPlanner>    motion_planner;
};


static map<int, shared_ptr<Handle> > handle_map;


int expo_create(FILE * msg)
{
  shared_ptr<Handle> handle(new Handle());
  
  HAL * hal(0);
  int front_sick_channel(0);
  double front_sick_x(0.15);
  double front_sick_y(0);
  double front_sick_theta(M_PI/2);
  int sick_nscans(361);
  double sick_rhomax(8);
  double sick_phi0(-M_PI/2);
  double sick_phirange(M_PI);
  handle->front_sick.reset(new Scanner(hal, front_sick_channel,
				       "front_sick",
				       Frame(front_sick_x,
					     front_sick_y,
					     front_sick_theta),
				       sick_nscans,
				       sick_rhomax,
				       sick_phi0,
				       sick_phirange));
  
  int rear_sick_channel(1);
  double rear_sick_x(-0.3);
  double rear_sick_y(0);
  double rear_sick_theta(-M_PI/2);
  handle->rear_sick.reset(new Scanner(hal, rear_sick_channel,
				      "rear_sick",
				      Frame(rear_sick_x,
					    rear_sick_y,
					    rear_sick_theta),
				      sick_nscans,
				      sick_rhomax,
				      sick_phi0,
				      sick_phirange));
  
  double wheelbase(0.6);
  double wheelradius(0.09);
  handle->drive.reset(new DiffDrive(hal, wheelbase, wheelradius));
  
  double timestep(0.1);
  double security_distance(0.05);
  double qd_max(0.1);
  double qdd_max(0.1);
  double sd_max(0.3);
  double thetad_max(0.1);
  double sdd_max(0.75 * wheelradius * qdd_max);
  double thetadd_max(1.5 * wheelradius * qdd_max / wheelbase);
  RobotModel::Parameters rm_parameters(security_distance,
				       wheelbase,
				       wheelradius,
				       qd_max,
				       qdd_max,
				       sd_max,
				       thetad_max,
				       sdd_max,
				       thetadd_max);
  Polygon poly;
  double x0(-0.3);
  double y0(-0.2);
  double x1(0.3);
  double y1(0.2);
  poly.AddPoint(x0, y0);
  poly.AddPoint(x1, y0);
  poly.AddPoint(x1, y1);
  poly.AddPoint(x0, y1);
  shared_ptr<Hull> hull(new Hull());
  hull->AddPolygon(poly);
  handle->robot_model.reset(new RobotModel(timestep, rm_parameters, hull));
  
  handle->motion_controller.reset(new MotionController("motion_controller",
						       *handle->robot_model,
						       *handle->drive));

  int dwa_dimension(41);
  double dwa_grid_width(2.2);
  double dwa_grid_height(1.5);
  double dwa_grid_resolution(0.03);
  double dwa_alpha_distance(0.5);
  double dwa_alpha_heading(0.1);
  double dwa_alpha_speed(0.1);
  handle->dynamic_window.reset(new DynamicWindow(dwa_dimension,
						 dwa_grid_width,
						 dwa_grid_height,
						 dwa_grid_resolution,
						 *handle->robot_model,
						 *handle->motion_controller,
						 dwa_alpha_distance,
						 dwa_alpha_heading,
						 dwa_alpha_speed));
  
  handle->odometry.reset(new Odometry(hal));
  
  double bb_shortpath(4);
  double bb_longpath(8);
  double bb_max_ignore_distance(4);
  sfl::BubbleList::Parameters bb_parameters(bb_shortpath,
					    bb_longpath,
					    bb_max_ignore_distance);
  handle->bubble_band.reset(new BubbleBand(*handle->robot_model,
					   *handle->odometry,
					   bb_parameters));
  
  handle->multiscanner.reset(new Multiscanner());
  handle->multiscanner->Add(handle->front_sick);
  handle->multiscanner->Add(handle->rear_sick);
  
  handle->motion_planner.reset(new MotionPlanner(*handle->motion_controller,
						 *handle->dynamic_window,
						 *handle->multiscanner,
						 *handle->robot_model,
						 *handle->bubble_band,
						 *handle->odometry));
  
  int result(0);
  if( ! handle_map.empty())
    result = handle_map.rbegin()->first + 1;
  
  handle_map.insert(make_pair(result, handle));
  return result;
}


int expo_set_goal(int handle,
		  double x,
		  double y,
		  double theta,
		  double dr,
		  double dtheta,
		  int viaGoal)
{
  return -2;
}


int expo_goal_reached(int handle)
{
  return -2;
}


int expo_update_dwa(int handle)
{
  return -2;
}


int expo_update_bband(int handle)
{
  return -2;
}


void expo_destroy(int handle)
{
}

