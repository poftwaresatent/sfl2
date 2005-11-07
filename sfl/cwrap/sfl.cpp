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


#include "sfl.h"
#include "Handlemap.hpp"
#include "cwrapHAL.hpp"
#include <sfl/api/Scanner.hpp>
#include <sfl/api/DiffDrive.hpp>
#include <sfl/api/RobotModel.hpp>
#include <sfl/dwa/DynamicWindow.hpp>
#include <sfl/bband/BubbleBand.hpp>


using boost::shared_ptr;


namespace sfl_cwrap {
  
  static Handlemap<sfl::HAL>           HAL_map;
  static Handlemap<sfl::Scanner>       Scanner_map;
  static Handlemap<sfl::DiffDrive>     DiffDrive_map;
  static Handlemap<sfl::RobotModel>    RobotModel_map;
  static Handlemap<sfl::DynamicWindow> DynamicWindow_map;
  static Handlemap<sfl::BubbleBand>    BubbleBand_map;
  
  shared_ptr<sfl::HAL>           get_HAL(int handle)
  { return HAL_map.Find(handle); }
  
  shared_ptr<sfl::Scanner>       get_Scanner(int handle)
  { return Scanner_map.Find(handle); }
  
  shared_ptr<sfl::DiffDrive>     get_DiffDrive(int handle)
  { return DiffDrive_map.Find(handle); }
  
  shared_ptr<sfl::RobotModel>    get_RobotModel(int handle)
  { return RobotModel_map.Find(handle); }
  
  shared_ptr<sfl::DynamicWindow> get_DynamicWindow(int handle)
  { return DynamicWindow_map.Find(handle); }
  
  shared_ptr<sfl::BubbleBand>    get_BubbleBand(int handle)
  { return BubbleBand_map.Find(handle); }


int sfl_create_HAL(struct cwrap_hal_s * cwrap_hal)
{
  shared_ptr<sfl_cwrap::cwrapHAL> hal(new sfl_cwrap::cwrapHAL(cwrap_hal));
  return HAL_map.Insert(hal);
}


void sfl_destroy_HAL(int handle)
{
  HAL_map.Erase(handle);
}

  
int sfl_create_Scanner(int hal_handle, int hal_channel,
		       double mount_x, double mount_y, double mount_theta,
		       int nscans, double rhomax, double phi0,
		       double phirange);

void sfl_destroy_Scanner(int handle);


int sfl_create_DiffDrive(double wheelbase, double wheelradius);

void sfl_destroy_DiffDrive(int handle);


int sfl_create_RobotModel(double timestep, double security_distance,
			  double qd_max, double qdd_max,
			  double sd_max, double thetad_max,
			  double sdd_max, double thetadd_max,
			  double * hull_x, double * hull_y, int hull_len);

void sfl_destroy_RobotModel(int handle);


int sfl_create_DynamicWindow(int dimension,
			     double grid_width,
			     double grid_height,
			     double grid_resolution,
			     double alpha_distance,
			     double alpha_heading,
			     double alpha_speed);

void sfl_destroy_DynamicWindow(int handle);


int sfl_create_BubbleBand(double shortpath, double longpath,
			  double max_ignore_distance);

void sfl_destroy_BubbleBand(int handle);
  
}  
