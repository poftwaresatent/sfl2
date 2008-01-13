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


#include "sfl.h"
#include "expo.h"
#include "Handlemap.hpp"
#include "cwrapHAL.hpp"
#include "../api/Scanner.hpp"
#include "../api/Multiscanner.hpp"
#include "../api/RobotModel.hpp"
#include "../api/Odometry.hpp"
#include "../util/Pthread.hpp"
#include "../dwa/DynamicWindow.hpp"
#include "../dwa/DistanceObjective.hpp"
#include "../dwa/HeadingObjective.hpp"
#include "../dwa/SpeedObjective.hpp"
#include "../bband/BubbleBand.hpp"
#include "../expo/MotionController.hpp"
#include <sstream>
#include <iostream>
#include <fstream>


using namespace sfl;
using namespace boost;
using namespace std;


namespace sfl_cwrap {


  static Handlemap<HAL>           HAL_map;
  static Handlemap<Scanner>       Scanner_map;
  static Handlemap<Multiscanner>  Multiscanner_map;
  static Handlemap<RobotModel>    RobotModel_map;
  static Handlemap<LegacyDynamicWindow> DynamicWindow_map;
  static Handlemap<BubbleBand>    BubbleBand_map;
  static Handlemap<Odometry>      Odometry_map;


  shared_ptr<HAL> get_HAL(int handle)
  { return HAL_map.Find(handle); }


  shared_ptr<Scanner> get_Scanner(int handle)
  { return Scanner_map.Find(handle); }


  shared_ptr<Multiscanner> get_Multiscanner(int handle)
  { return Multiscanner_map.Find(handle); }


  shared_ptr<RobotModel> get_RobotModel(int handle)
  { return RobotModel_map.Find(handle); }


  shared_ptr<LegacyDynamicWindow> get_DynamicWindow(int handle)
  { return DynamicWindow_map.Find(handle); }


  shared_ptr<BubbleBand> get_BubbleBand(int handle)
  { return BubbleBand_map.Find(handle); }


  shared_ptr<Odometry> get_Odometry(int handle)
  { return Odometry_map.Find(handle); }

}


using namespace sfl_cwrap;


int sfl_create_HAL(struct cwrap_hal_s * cwrap_hal)
{ return HAL_map.InsertRaw(new cwrapHAL(cwrap_hal)); }


int sfl_create_Scanner(int hal_handle, int hal_channel,
		       double mount_x, double mount_y, double mount_theta,
		       int nscans, double rhomax, double phi0,
		       double phirange)
{
//   shared_ptr<Odometry> odo(get_Odometry(odometry_handle));
//   if( ! odo)
//     return -1;
  shared_ptr<HAL> hal(get_HAL(hal_handle));
  if( ! hal)
    return -2;
  ostringstream os;
  os << "cwrap-Scanner-" << hal_channel;
  shared_ptr<Mutex> mutex(Mutex::Create(os.str()));
  if( ! mutex)
    return -3;
  return Scanner_map.
    InsertRaw(new Scanner(hal, hal_channel,
			  Frame(mount_x, mount_y, mount_theta),
			  nscans, rhomax, phi0, phirange, mutex));
}


int sfl_create_Multiscanner(int odometry_handle,
			    int * scanner_handle, int nscanners)
{
  shared_ptr<Odometry> odo(get_Odometry(odometry_handle));
  if( ! odo)
    return -1;
  shared_ptr<Multiscanner> ms(new Multiscanner(odo));
  for(int is(0); is < nscanners; ++is){
    shared_ptr<Scanner> sc(get_Scanner(scanner_handle[is]));
    if( ! sc)
      return -2 - is;
    ms->Add(sc);
  }
  return Multiscanner_map.Insert(ms);
}


int sfl_create_BubbleBand(int RobotModel_handle,
			  int Odometry_handle,
			  int Multiscanner_handle,
			  double shortpath, double longpath,
			  double max_ignore_distance)
{
  shared_ptr<RobotModel> rm(get_RobotModel(RobotModel_handle));
  if( ! rm)
    return -1;
  shared_ptr<Odometry> odom(get_Odometry(Odometry_handle));
  if( ! odom)
    return -2;
  shared_ptr<Multiscanner> mscan(get_Multiscanner(Multiscanner_handle));
  if( ! mscan)
    return -3;
  shared_ptr<RWlock> rwlock(RWlock::Create("cwrap-BubbleBand"));
  if( ! rwlock)
    return -4;
  BubbleList::Parameters parms(shortpath, longpath, max_ignore_distance);
  return BubbleBand_map.InsertRaw(new BubbleBand(*rm, *odom, *mscan,
						 parms, rwlock));
}


int sfl_create_RobotModel(double security_distance,
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
  return RobotModel_map.InsertRaw(new RobotModel(parms, hull));
}


int sfl_create_DynamicWindow(int RobotModel_handle,
			     int MotionController_handle,
			     int dimension,
			     double grid_width,
			     double grid_height,
			     double grid_resolution,
			     double alpha_distance,
			     double alpha_heading,
			     double alpha_speed,
			     const char * filename)
{
  shared_ptr<RobotModel> rm(get_RobotModel(RobotModel_handle));
  if( ! rm)
    return -1;
  shared_ptr<MotionController>
    mc(get_MotionController(MotionController_handle));
  if( ! mc)
    return -2;
  shared_ptr<LegacyDynamicWindow> dwa(new LegacyDynamicWindow(dimension,
						  grid_width,
						  grid_height,
						  grid_resolution,
						  rm,
						  *mc,
						  alpha_distance,
						  alpha_heading,
						  alpha_speed,
						  false));
  shared_ptr<ofstream> osf;
  ostream * osp(0);
  if(0 != filename){
    if("" == string(filename))
      osp = &cerr;
    else{
      osf.reset(new ofstream(filename));
      osp = osf.get();
    }
  }
  if( ! dwa->Initialize(osp, false))
    return -3;
  return DynamicWindow_map.Insert(dwa);
}


int sfl_create_Odometry(int HAL_handle)
{
  shared_ptr<HAL> hal(get_HAL(HAL_handle));
  if( ! hal)
    return -1;
  shared_ptr<RWlock> rwlock(RWlock::Create("cwrap-Odometry"));
  if( ! rwlock)
    return -2;
  return Odometry_map.InsertRaw(new Odometry(hal, rwlock));
}


void sfl_destroy_HAL(int handle)
{ HAL_map.Erase(handle); }


void sfl_destroy_Scanner(int handle)
{ Scanner_map.Erase(handle); }


void sfl_destroy_RobotModel(int handle)
{ RobotModel_map.Erase(handle); }


void sfl_destroy_DynamicWindow(int handle)
{ DynamicWindow_map.Erase(handle); }


void sfl_destroy_BubbleBand(int handle)
{ BubbleBand_map.Erase(handle); }


void sfl_destroy_Odometry(int handle)
{ Odometry_map.Erase(handle); }


void sfl_destroy_Multiscanner(int handle)
{ Multiscanner_map.Erase(handle); }


int sfl_dump_obstacles(int DynamicWindow_handle, const char * filename,
		       const char * prefix)
{
  shared_ptr<LegacyDynamicWindow> dwa(get_DynamicWindow(DynamicWindow_handle));
  if( ! dwa)
    return -1;
  shared_ptr<ofstream> osf;
  ostream * osp(&cout);
  if((0 != filename) && ("" != string(filename))){
    osf.reset(new ofstream(filename));
    osp = osf.get();
  }
  dwa->DumpObstacles(*osp, prefix);
  return 0;
}


int sfl_dump_dwa(int DynamicWindow_handle, const char * filename,
		 const char * prefix)
{
  shared_ptr<LegacyDynamicWindow> dwa(get_DynamicWindow(DynamicWindow_handle));
  if( ! dwa)
    return -1;
  shared_ptr<ofstream> osf;
  ostream * osp(&cout);
  if((0 != filename) && ("" != string(filename))){
    osf.reset(new ofstream(filename));
    osp = osf.get();
  }
  dwa->DumpObjectives(*osp, prefix);
  return 0;
}
