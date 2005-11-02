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
using sfl::Goal;
using expo::MotionPlanner;
using expo::MotionController;
using boost::scoped_ptr;
using boost::shared_ptr;
using std::map;
using std::make_pair;


class CWrapHAL: public HAL {
public:
  CWrapHAL(struct cwrap_hal_s * hal) : m_hal(hal) { }
  
  virtual int time_get(struct ::timespec * stamp)
  { return m_hal->time_get(stamp); }
  
  virtual int odometry_set(double x, double y, double theta,
			   double sxx, double syy, double stt,
			   double sxy, double sxt, double syt)
  { return m_hal->odometry_set(x, y, theta,
			       sxx, syy, stt,
			       sxy, sxt, syt); }
  
  virtual int odometry_get(struct ::timespec * stamp,
			   double * x, double * y, double * theta,
			   double * sxx, double * syy, double * stt,
			   double * sxy, double * sxt, double * syt)
  { return m_hal->odometry_get(stamp, x, y, theta,
			       sxx, syy, stt,
			       sxy, sxt, syt); }
  
  virtual int speed_set(double qdl, double qdr)
  { return m_hal->speed_set(qdl, qdr); }
  
  virtual int scan_get(int channel, double * rho, int rho_len,
		       struct ::timespec * t0, struct ::timespec * t1)
  { return m_hal->scan_get(channel, rho, rho_len, t0, t1); }
  
private:
  struct cwrap_hal_s * m_hal;
};


class Handle {
public:
  scoped_ptr<CWrapHAL>         hal;
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


typedef map<int, shared_ptr<Handle> > handle_map_t;


static handle_map_t handle_map;


int expo_create(struct cwrap_hal_s * cwrap_hal,
		FILE * msg)
{
  shared_ptr<Handle> handle(new Handle());
  
  handle->hal.reset(new CWrapHAL(cwrap_hal));
  
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
  handle_map_t::iterator ih(handle_map.find(handle));
  if(handle_map.end() == ih)
    return -1;
  ih->second->motion_planner->SetGoal(Goal(x, y, theta, dr, dtheta, viaGoal));
  return 0;
}


int expo_goal_reached(int handle)
{
  handle_map_t::iterator ih(handle_map.find(handle));
  if(handle_map.end() == ih)
    return -1;
  return ih->second->motion_planner->GoalReached() ? 1 : 0;
}


int expo_update_all(int handle)
{
  handle_map_t::iterator ih(handle_map.find(handle));
  if(handle_map.end() == ih)
    return -1;
  if(0 != ih->second->odometry->Update())
    return -2;
  if(0 != ih->second->front_sick->Update())
    return -3;
  if(0 != ih->second->rear_sick->Update())
    return -4;
  ih->second->motion_planner->Update();
  if(0 != ih->second->motion_controller->Update())
    return -5;
  return 0;
}


void expo_destroy(int handle)
{
  handle_map_t::iterator ih(handle_map.find(handle));
  if(handle_map.end() != ih)
    handle_map.erase(ih);
}
