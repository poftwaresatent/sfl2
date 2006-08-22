/*
 * Copyright (c) 2005 CNRS/LAAS
 *
 * Author: Roland Philippsen <roland dot philippsen at gmx dot net>
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


#ifndef CWRAP_EXPO_H
#define CWRAP_EXPO_H

#ifdef __cplusplus
# include <boost/shared_ptr.hpp>
namespace expo {
  class MotionController;
  class MotionPlanner;
}
namespace sfl_cwrap {
  boost::shared_ptr<expo::MotionController> get_MotionController(int handle);
  boost::shared_ptr<expo::MotionPlanner>    get_MotionPlanner(int handle);
}
extern "C" {
#endif // __cplusplus
  
#include <sfl/cwrap/hal.h>
#include <stdio.h>
  
  
  /** retval of expo_get_state() */
  enum EXPO_CWRAP_STATE_ID {
    EXPO_CWRAP_INVALID             = -2,
    EXPO_CWRAP_NULL                =  0,
    EXPO_CWRAP_TAKE_AIM            =  1,
    EXPO_CWRAP_AIMED               =  2,
    EXPO_CWRAP_ADJUST_GOAL_HEADING =  3,
    EXPO_CWRAP_AT_GOAL             =  4
  };
  
  
  /** \return >=0 on success, -1 if invalid RobotModel_handle, -2 if
      invalid HAL_handle, -3 if could not allocate
      sfl::Mutex. */
  int expo_create_MotionController(int RobotModel_handle,
				   int HAL_handle);
  
  int expo_create_MotionPlanner(int MotionController_handle,
				int DynamicWindow_handle,
				int Multiscanner_handle,
				int RobotModel_handle,
				int BubbleBand_handle,
				int Odometry_handle);
  
  /** \return a MotionPlanner handle */
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
		   double bb_max_ignore_distance);
  
  
  /**
     Define the robot's goal in absolute coordinates (world frame).
     
     \return 0 on success, -1 if invalid handle
  */
  int expo_set_goal(int MotionPlanner_handle,
		    /** x-coordinate [m] */
		    double x,
		    /** y-coordinate [m] */
		    double y,
		    /** heading angle [rad] */
		    double theta,
		    /** goal radius [m] */
		    double dr,
		    /** angular precision [rad] */
		    double dtheta,
		    /** if !=0 then the robot won't slow down inside
			the goal radius; usefull only if you
			immediately set the next goal as soon as this
			one is reached */
		    int viaGoal);
  
  
  /**
     \return 0 if the goal has NOT been reached, 1 if the robot has
     reached the goal, -1 if the handle is invalid
  */
  int expo_goal_reached(/** the handle from expo_create_MotionPlanner() */
			int MotionPlanner_handle);
  
  
  /**
     Perform one cycle of overall algorithm updates, which includes
     the bubble band (and NF1 if necessary) and the reactive obstacle
     avoidance. At the end, new motion commands are sent to the motor
     controllers.
     
     \todo Separate the three different task levels: DWA (RT 10Hz),
     Bubble Band (non-RT 5Hz), NF1 (non-RT on-demand).
     
     \return
     <ul><li>  0 success </li>
         <li> -1 invalid handle </li>
         <li> -2 odometry update error </li>
         <li> -3 (multi)scanner update error </li>
         <li> -4 motion controller update error </li></ul>
  */
  int expo_update_all(/** the handle from expo_create_MotionPlanner() */
		      int MotionPlanner_handle,
		      /** (expected/fixed) delay until the next invokation */
		      double timestep);
  
  /* \return
     <ul><li> -1: invalid handle </li>
         <li> -2: EXPO_CWRAP_INVALID </li>
         <li>  0: EXPO_CWRAP_NULL </li>
         <li>  1: EXPO_CWRAP_TAKE_AIM </li>
         <li>  2: EXPO_CWRAP_AIMED </li>
         <li>  3: EXPO_CWRAP_ADJUST_GOAL_HEADING </li>
         <li>  4: EXPO_CWRAP_AT_GOAL </li></ul>
  */
  int expo_get_state(/** the handle from expo_create_MotionPlanner() */
		     int MotionPlanner_handle);
  
  
  /** \note Invalid handles are silently ignored. */
  void expo_destroy_MotionController(int handle);
  
  /** \note Invalid handles are silently ignored. */
  void expo_destroy_MotionPlanner(int handle);
  
  
#ifdef __cplusplus
} // extern "C"
#endif // __cplusplus

#endif // CWRAP_EXPO_H
