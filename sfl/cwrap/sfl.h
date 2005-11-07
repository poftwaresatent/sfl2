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


#ifndef CWRAP_SFL_H
#define CWRAP_SFL_H

#ifdef __cplusplus
# include <boost/shared_ptr.hpp>
namespace sfl {
  class HAL;
  class Scanner;
  class DiffDrive;
  class RobotModel;
  class DynamicWindow;
  class BubbleBand;
  class Odometry;
}
namespace sfl_cwrap {
  boost::shared_ptr<sfl::HAL>           get_HAL(int handle);
  boost::shared_ptr<sfl::Scanner>       get_Scanner(int handle);
  boost::shared_ptr<sfl::DiffDrive>     get_DiffDrive(int handle);
  boost::shared_ptr<sfl::RobotModel>    get_RobotModel(int handle);
  boost::shared_ptr<sfl::DynamicWindow> get_DynamicWindow(int handle);
  boost::shared_ptr<sfl::BubbleBand>    get_BubbleBand(int handle);
  boost::shared_ptr<sfl::Odometry>      get_Odometry(int handle);
}
extern "C" {
#endif // __cplusplus
  
#include <sfl/cwrap/hal.h>
  
  
  int sfl_create_HAL(struct cwrap_hal_s * cwrap_hal);
  
  int sfl_create_Scanner(int hal_handle, int hal_channel, const char * name,
			 double mount_x, double mount_y, double mount_theta,
			 int nscans, double rhomax, double phi0,
			 double phirange);
  
  int sfl_create_DiffDrive(int hal_handle,
			   double wheelbase, double wheelradius);
  
  int sfl_create_RobotModel(double timestep, double security_distance,
			    double wheelbase, double wheelradius,
			    double qd_max, double qdd_max,
			    double sd_max, double thetad_max,
			    double sdd_max, double thetadd_max,
			    double * hull_x, double * hull_y, int hull_len);
  
  int sfl_create_DynamicWindow(int RobotModel_handle,
			       /** \note expo_create_MotionController() */
			       int MotionController_handle,
			       int dimension,
			       double grid_width,
			       double grid_height,
			       double grid_resolution,
			       double alpha_distance,
			       double alpha_heading,
			       double alpha_speed);
  
  int sfl_create_BubbleBand(int RobotModel_handle, int Odometry_handle,
			    double shortpath, double longpath,
			    double max_ignore_distance);
  
  int sfl_create_Odometry(int HAL_handle);
  
  
  void sfl_destroy_HAL(int handle);
  void sfl_destroy_Scanner(int handle);
  void sfl_destroy_DiffDrive(int handle);
  void sfl_destroy_RobotModel(int handle);
  void sfl_destroy_DynamicWindow(int handle);
  void sfl_destroy_BubbleBand(int handle);
  void sfl_destroy_Odometry(int handle);
  
  
#ifdef __cplusplus
} // extern "C"
#endif // __cplusplus

#endif // CWRAP_SFL_H