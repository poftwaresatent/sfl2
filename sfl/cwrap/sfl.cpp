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
#include "expo.h"
#include "Handlemap.hpp"
#include "cwrapHAL.hpp"
#include <sfl/api/Scanner.hpp>
#include <sfl/api/Multiscanner.hpp>
#include <sfl/api/DiffDrive.hpp>
#include <sfl/api/RobotModel.hpp>
#include <sfl/api/Odometry.hpp>
#include <sfl/dwa/DynamicWindow.hpp>
#include <sfl/bband/BubbleBand.hpp>
#include <sfl/expo/MotionController.hpp>


using namespace sfl;
using boost::shared_ptr;


namespace sfl_cwrap {
  

  static Handlemap<HAL>           HAL_map;
  static Handlemap<Scanner>       Scanner_map;
  static Handlemap<Multiscanner>  Multiscanner_map;
  static Handlemap<DiffDrive>     DiffDrive_map;
  static Handlemap<RobotModel>    RobotModel_map;
  static Handlemap<DynamicWindow> DynamicWindow_map;
  static Handlemap<BubbleBand>    BubbleBand_map;
  static Handlemap<Odometry>      Odometry_map;

  
  shared_ptr<HAL> get_HAL(int handle)
  { return HAL_map.Find(handle); }
  
  
  shared_ptr<Scanner> get_Scanner(int handle)
  { return Scanner_map.Find(handle); }
  
  
  shared_ptr<Multiscanner> get_Multiscanner(int handle)
  { return Multiscanner_map.Find(handle); }

  
  shared_ptr<DiffDrive> get_DiffDrive(int handle)
  { return DiffDrive_map.Find(handle); }

  
  shared_ptr<RobotModel> get_RobotModel(int handle)
  { return RobotModel_map.Find(handle); }

  
  shared_ptr<DynamicWindow> get_DynamicWindow(int handle)
  { return DynamicWindow_map.Find(handle); }

  
  shared_ptr<BubbleBand> get_BubbleBand(int handle)
  { return BubbleBand_map.Find(handle); }

  
  shared_ptr<Odometry> get_Odometry(int handle)
  { return Odometry_map.Find(handle); }
  
  
  int sfl_create_HAL(struct cwrap_hal_s * cwrap_hal)
  { return HAL_map.InsertRaw(new cwrapHAL(cwrap_hal)); }
  
  
  int sfl_create_Scanner(int hal_handle, int hal_channel, const char * name,
			 double mount_x, double mount_y, double mount_theta,
			 int nscans, double rhomax, double phi0,
			 double phirange)
  {
    shared_ptr<HAL> hal(get_HAL(hal_handle));
    if( ! hal)
      return -1;
    return Scanner_map.InsertRaw(new Scanner(hal.get(), hal_channel, name,
					     Frame(mount_x,
						   mount_y,
						   mount_theta),
					     nscans, rhomax, phi0, phirange));
  }
  
  
  int sfl_create_Multiscanner(int * Scanner_handle, int nscanners)
  {
    shared_ptr<Multiscanner> ms(new Multiscanner());
    for(int is(0); is < nscanners; ++is){
      shared_ptr<Scanner> sc(get_Scanner(Scanner_handle[is]));
      if( ! sc)
	return -1;
      ms->Add(sc);
    }
    return Multiscanner_map.Insert(ms);
  }
  
  
  int sfl_create_BubbleBand(int RobotModel_handle,
			    int Odometry_handle,
			    double shortpath, double longpath,
			    double max_ignore_distance)
  {
    shared_ptr<RobotModel> rm(get_RobotModel(RobotModel_handle));
    if( ! rm)
      return -1;
    shared_ptr<Odometry> odom(get_Odometry(Odometry_handle));
    if( ! odom)
      return -2;
    BubbleList::Parameters parms(shortpath, longpath, max_ignore_distance);
    return BubbleBand_map.InsertRaw(new BubbleBand(*rm, *odom, parms));
  }
  
  
  int sfl_create_DiffDrive(int hal_handle,
			   double wheelbase, double wheelradius)
  {
    shared_ptr<HAL> hal(get_HAL(hal_handle));
    if( ! hal)
      return -1;
    return DiffDrive_map.InsertRaw(new DiffDrive(hal.get(),
						 wheelbase, wheelradius));
  }
  
  
  int sfl_create_RobotModel(double timestep, double security_distance,
			    double wheelbase, double wheelradius,
			    double qd_max, double qdd_max,
			    double sd_max, double thetad_max,
			    double sdd_max, double thetadd_max,
			    double * hull_x, double * hull_y, int hull_len)
  {
    RobotModel::Parameters parms(security_distance, wheelbase, wheelradius,
				 qd_max, qdd_max, sd_max, thetad_max,
				 sdd_max, thetadd_max);
    Polygon poly;
    for(int ip(0); ip < hull_len; ++ip)
      poly.AddPoint(hull_x[ip], hull_y[ip]);
    shared_ptr<Hull> hull(new Hull());
    hull->AddPolygon(poly);
    return RobotModel_map.InsertRaw(new RobotModel(timestep, parms, hull));
  }
  
  
  int sfl_create_DynamicWindow(int RobotModel_handle,
			       int MotionController_handle,
			       int dimension,
			       double grid_width,
			       double grid_height,
			       double grid_resolution,
			       double alpha_distance,
			       double alpha_heading,
			       double alpha_speed)
  {
    shared_ptr<RobotModel> rm(get_RobotModel(RobotModel_handle));
    if( ! rm)
      return -1;
    shared_ptr<MotionController>
      mc(get_MotionController(MotionController_handle));
    if( ! mc)
      return -2;
    return DynamicWindow_map.InsertRaw(new DynamicWindow(dimension,
							 grid_width,
							 grid_height,
							 grid_resolution,
							 *rm,
							 *mc,
							 alpha_distance,
							 alpha_heading,
							 alpha_speed));
  }
  
  
  void sfl_destroy_HAL(int handle)
  { HAL_map.Erase(handle); }
  
  
  void sfl_destroy_Scanner(int handle)
  { Scanner_map.Erase(handle); }
  
  
  void sfl_destroy_DiffDrive(int handle)
  { DiffDrive_map.Erase(handle); }
  
  
  void sfl_destroy_RobotModel(int handle)
  { RobotModel_map.Erase(handle); }
  
  
  void sfl_destroy_DynamicWindow(int handle)
  { DynamicWindow_map.Erase(handle); }
  
  
  void sfl_destroy_BubbleBand(int handle)
  { BubbleBand_map.Erase(handle); }
  
  
  void sfl_destroy_Odometry(int handle)
  { Odometry_map.Erase(handle); }
  
}  
