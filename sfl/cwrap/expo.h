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


#ifndef CWRAP_EXPO_H
#define CWRAP_EXPO_H

#ifdef __cplusplus
extern "C" {
#endif // __cplusplus
  
#include <sfl/cwrap/hal.h>
#include <stdio.h>
  

  /**
     Create the objects necessary to use the Expo.02 obstacle avoidance
     system using C language bindings.

     \return >=0 on success: handle that refers to an internal
     database. -1 on failure, with an error message written to the
     provided FILE*.
   
     \note Use expo_destroy() to release the handle after you've
     finished using the Expo.02 obstacle avoidance system.
  */
  int expo_create(/** hardware abstraction layer for the expo system */
		  struct cwrap_hal_s * cwrap_hal,
		  /** for error messages */
		  FILE * msg);
  
  
  /**
     Define the robot's goal in absolute coordinates (world frame).
     
     \return 0 on success, -1 if invalid handle
  */
  int expo_set_goal(/** the handle obtained from expo_create() */
		    int handle,
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
  int expo_goal_reached(/** the handle obtained from expo_create() */
			int handle);
  
  
  /**
     Perform one cycle of Dynamic Window updates, ie the reactive
     obstacle avoidance. This should be done in a real-time loop that
     runs at 10Hz or so.
     
     \return 0 on success, -1 if the handle is invalid
  */
  int expo_update_dwa(/** the handle obtained from expo_create() */
		      int handle);

  
  /**
     Perform one cycle of Bubble Band updates, ie the path
     modification algorithm that adapts the trajectory to changes in
     the environment. It does not have to be real-time, but should be
     called periodically with a frequency comparable to that of
     expo_update_dwa().
     
     \return 0 on success, -1 if the handle is invalid
  */
  int expo_update_bband(/** the handle obtained from expo_create() */
			int handle);
  
  
  /**
     \note Invalid handles are silently ignored.
  */
  void expo_destroy(/** the handle obtained from expo_create() */
		    int handle);
  
  
#ifdef __cplusplus
} // extern "C"
#endif // __cplusplus

#endif // CWRAP_EXPO_H
