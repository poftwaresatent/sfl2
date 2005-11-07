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
  
  
  class Handle {
  public:
    shared_ptr<HAL>              hal;
    shared_ptr<Scanner>          front_sick;
    shared_ptr<Scanner>          rear_sick;
    shared_ptr<DiffDrive>        drive;
    shared_ptr<RobotModel>       robot_model;
    shared_ptr<MotionController> motion_controller;
    shared_ptr<DynamicWindow>    dynamic_window;
    shared_ptr<Odometry>         odometry;
    shared_ptr<BubbleBand>       bubble_band;
    shared_ptr<Multiscanner>     multiscanner;
    shared_ptr<MotionPlanner>    motion_planner;
  };
  
  
  static Handlemap<Handle> handlemap;
  
  
  static Handlemap<MotionController> MotionController_map;
  static Handlemap<MotionPlanner>    MotionPlanner_map;
  
  
  shared_ptr<MotionController> get_MotionController(int handle)
  { return MotionController_map.Find(handle); }
  
  
  shared_ptr<MotionPlanner>    get_MotionPlanner(int handle)
  { return MotionPlanner_map.Find(handle); }
  
  
  int expo_create(int hal_handle)
  {
    shared_ptr<Handle> handle(new Handle());
  
    handle->hal = get_HAL(hal_handle);
    if( ! handle->hal)
      return -1;
  
    int front_sick_channel(0);
    double front_sick_x(0.15);
    double front_sick_y(0);
    double front_sick_theta(M_PI/2);
    int sick_nscans(361);
    double sick_rhomax(8);
    double sick_phi0(-M_PI/2);
    double sick_phirange(M_PI);
    handle->front_sick.reset(new Scanner(handle->hal.get(), front_sick_channel,
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
    handle->rear_sick.reset(new Scanner(handle->hal.get(), rear_sick_channel,
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
    handle->drive.reset(new DiffDrive(handle->hal.get(),
				      wheelbase, wheelradius));
  
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
  
    handle->odometry.reset(new Odometry(handle->hal.get()));
  
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
  
    return handlemap.Insert(handle);
  }


  int expo_set_goal(int handle,
		    double x,
		    double y,
		    double theta,
		    double dr,
		    double dtheta,
		    int viaGoal)
  {
    shared_ptr<Handle> h(handlemap.Find(handle));
    if( ! h)
      return -1;
    h->motion_planner->SetGoal(Goal(x, y, theta, dr, dtheta, viaGoal));
    return 0;
  }


  int expo_goal_reached(int handle)
  {
    shared_ptr<Handle> h(handlemap.Find(handle));
    if( ! h)
      return -1;
    return h->motion_planner->GoalReached() ? 1 : 0;
  }


  int expo_update_all(int handle)
  {
    shared_ptr<Handle> h(handlemap.Find(handle));
    if( ! h)
      return -1;
    if(0 != h->odometry->Update())
      return -2;
    if(0 != h->front_sick->Update())
      return -3;
    if(0 != h->rear_sick->Update())
      return -4;
    h->motion_planner->Update();
    if(0 != h->motion_controller->Update())
      return -5;
    return 0;
  }


  void expo_destroy(int handle)
  {
    handlemap.Erase(handle);
  }

}
