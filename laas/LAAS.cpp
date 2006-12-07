/* 
 * Copyright (C) 2006 Roland Philippsen <roland dot philippsen at gmx dot net>
 * 
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 * 
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307
 * USA
 */


#include "LAAS.hpp"
#include <npm/robox/MPDrawing.hpp>
#include <npm/robox/OCamera.hpp>
#include <npm/robox/ODrawing.hpp>
#include <npm/robox/DODrawing.hpp>
#include <npm/robox/DWDrawing.hpp>
#include <npm/robox/RHDrawing.hpp>
#include <npm/robox/BBDrawing.hpp>
#include <npm/robox/GridLayerCamera.hpp>
#include <npm/robox/GridLayerDrawing.hpp>
#include <npm/robox/expoparams.hpp>
#include <npm/common/OdometryDrawing.hpp>
#include <npm/common/StillCamera.hpp>
#include <npm/common/HAL.hpp>
#include <npm/common/DiffDrive.hpp>
#include <npm/common/RobotDescriptor.hpp>
#include <npm/common/Lidar.hpp>
#include <sfl/util/pdebug.hpp>
#include <sfl/api/Odometry.hpp>
#include <sfl/api/Scanner.hpp>
#include <sfl/api/Multiscanner.hpp>
#include <sfl/api/Pose.hpp>
#include <sfl/api/RobotModel.hpp>
#include <sfl/api/Goal.hpp>
#include <sfl/dwa/DynamicWindow.hpp>
#include <sfl/bband/BubbleBand.hpp>
#include <sfl/expo/MotionPlanner.hpp>
#include <sfl/expo/MotionController.hpp>
#include <sfl/cwrap/hal.h>
#include <sfl/cwrap/sfl.h>
#include <sfl/cwrap/expo.h>
#include <iostream>
#include <sstream>
#include <map>


#define PDEBUG PDEBUG_OFF


using namespace npm;
using namespace sfl;
using namespace boost;
using namespace std;


static int cwrap_laas_time_get(struct cwrap_hal_s * self,
			       struct timespec * stamp);
static int cwrap_laas_odometry_set(struct cwrap_hal_s * self,
				   double x, double y, double theta,
				   double sxx, double syy, double stt,
				   double sxy, double sxt, double syt);
static int cwrap_laas_odometry_get(struct cwrap_hal_s * self,
				   struct timespec * stamp,
				   double * x, double * y, double * theta,
				   double * sxx, double * syy, double * stt,
				   double * sxy, double * sxt, double * syt);
static int cwrap_laas_speed_set(struct cwrap_hal_s * self,
				double qdl, double qdr);
static int cwrap_laas_speed_get(struct cwrap_hal_s * self,
				double * qdl, double * qdr);
static int cwrap_laas_scan_get(struct cwrap_hal_s * self,
			       int channel, double * rho, size_t * rho_len,
			       struct timespec * t0, struct timespec * t1);


static shared_ptr<Hull> create_jido_hull()
{
  shared_ptr<Hull> hull(new Hull());
  static const double xa(0.35);
  static const double ya(0.4);
  Polygon outline;
  outline.AddPoint( xa,  ya);
  outline.AddPoint(-xa,  ya);
  outline.AddPoint(-xa, -ya);
  outline.AddPoint( xa, -ya);
  hull->AddPolygon(outline);
  return hull;
}


static shared_ptr<Hull> create_rackham_hull()
{
  shared_ptr<Hull> hull(new Hull());
  static const double rr(0.26);
  static const int npoints(12);
  Polygon outline;
  for(int ii(0); ii < npoints; ++ii){
    const double phi(2 * ii * M_PI / npoints);
    outline.AddPoint(rr * cos(phi), rr * sin(phi));
  }
  hull->AddPolygon(outline);
  return hull;
}


typedef map<cwrap_hal_s *, shared_ptr<npm::HAL> > halmap_t;
static halmap_t halmap;


LAAS::
LAAS(shared_ptr<sfl::Hull> hull,
     shared_ptr<RobotDescriptor> descriptor, const World & world)
  : RobotClient(descriptor, world, true), m_hull(hull)
{
  for(size_t ip(0); ip < m_hull->GetNPolygons(); ++ip){
    shared_ptr<const Polygon> poly(m_hull->GetPolygon(ip));
    for(size_t il(0); il < poly->GetNPoints(); ++il)
      AddLine(*poly->_GetLine(il));
  }
  
  m_cwrap_hal.reset(new cwrap_hal_s());
  m_cwrap_hal->time_get     = cwrap_laas_time_get;
  m_cwrap_hal->odometry_set = cwrap_laas_odometry_set;
  m_cwrap_hal->odometry_get = cwrap_laas_odometry_get;
  m_cwrap_hal->speed_set    = cwrap_laas_speed_set;
  m_cwrap_hal->speed_get    = cwrap_laas_speed_get;
  m_cwrap_hal->scan_get     = cwrap_laas_scan_get;
  halmap[m_cwrap_hal.get()] = GetHAL();
  
  m_hal_handle = sfl_create_HAL(m_cwrap_hal.get());
  if(0 > m_hal_handle){
    fprintf(stderr,
	    "ERROR in LAAS ctor: sfl_create_HAL(): %d\n", m_hal_handle);
    exit(EXIT_FAILURE);
  }
    
  m_odometry_handle = sfl_create_Odometry(m_hal_handle);
  if(0 > m_odometry_handle){
    fprintf(stderr,
	    "ERROR in LAAS ctor: sfl_create_Odometry(): %d\n",
	    m_odometry_handle);
    exit(EXIT_FAILURE);
  }
  m_odometry = sfl_cwrap::get_Odometry(m_odometry_handle);
  
  expoparams params(descriptor);
  
  m_front_handle =
    sfl_create_Scanner(m_hal_handle,
		       params.front_channel,
		       params.front_mount_x,
		       params.front_mount_y,
		       params.front_mount_theta,
		       params.front_nscans,
		       params.front_rhomax,
		       params.front_phi0,
		       params.front_phirange);
  if(0 > m_front_handle){
    fprintf(stderr,
	    "ERROR in LAAS ctor: sfl_create_Scanner(front): %d\n",
	    m_front_handle);
    exit(EXIT_FAILURE);
  }
  m_front = sfl_cwrap::get_Scanner(m_front_handle);
  if( ! m_front){
    cerr << "BUG: invalid m_front just after creation via cwrap\n";
    exit(EXIT_FAILURE);
  }
  DefineLidar(m_front);
  
  m_drive = DefineDiffDrive(params.model_wheelbase, params.model_wheelradius);
  
  double tmp_hull_x[m_hull->GetNPoints()];
  double tmp_hull_y[m_hull->GetNPoints()];
  for(size_t ip(0); ip < m_hull->GetNPolygons(); ++ip){
    shared_ptr<const Polygon> poly(m_hull->GetPolygon(ip));
    for(size_t il(0); il < poly->GetNPoints(); ++il){
      shared_ptr<sfl::Line> line(poly->_GetLine(il));
      tmp_hull_x[il] = line->X0();
      tmp_hull_y[il] = line->Y0();
    }
  }
  m_robotModel_handle = sfl_create_RobotModel(params.model_security_distance,
					      params.model_wheelbase,
					      params.model_wheelradius,
					      params.model_qd_max,
					      params.model_qdd_max,
					      params.model_sd_max,
					      params.model_thetad_max,
					      params.model_sdd_max,
					      params.model_thetadd_max,
					      tmp_hull_x,
					      tmp_hull_y,
					      m_hull->GetNPoints());

  if(0 > m_robotModel_handle){
    fprintf(stderr,
	    "ERROR in LAAS ctor: sfl_create_RobotModel(): %d\n",
	    m_robotModel_handle);
    exit(EXIT_FAILURE);
  }
  m_robotModel = sfl_cwrap::get_RobotModel(m_robotModel_handle);
  
  m_motionController_handle = expo_create_MotionController(m_robotModel_handle,
							   m_hal_handle);
  if(0 > m_motionController_handle){
    fprintf(stderr, "ERROR in LAAS ctor:"
	    " sfl_create_MotionController(): %d\n", m_motionController_handle);
    exit(EXIT_FAILURE);
  }
  m_motionController =
    sfl_cwrap::get_MotionController(m_motionController_handle);
  
  m_dynamicWindow_handle =
    sfl_create_DynamicWindow(m_robotModel_handle,
			     m_motionController_handle,
			     params.dwa_dimension,
			     params.dwa_grid_width,
			     params.dwa_grid_height,
			     params.dwa_grid_resolution,
			     params.dwa_alpha_distance,
			     params.dwa_alpha_heading,
			     params.dwa_alpha_speed,
			     "/dev/stderr");
  if(0 > m_dynamicWindow_handle){
    fprintf(stderr,
	    "ERROR in LAAS ctor: sfl_create_DynamicWindow(): %d\n",
	    m_dynamicWindow_handle);
    exit(EXIT_FAILURE);
  }
  m_dynamicWindow = sfl_cwrap::get_DynamicWindow(m_dynamicWindow_handle);
  
  m_multiscanner_handle =
    sfl_create_Multiscanner(m_odometry_handle, &m_front_handle, 1);
  if(0 > m_multiscanner_handle){
    fprintf(stderr,
	    "ERROR in LAAS ctor: sfl_create_Multiscanner(): %d\n",
	    m_multiscanner_handle);
    exit(EXIT_FAILURE);
  }
  m_multiscanner = sfl_cwrap::get_Multiscanner(m_multiscanner_handle);
  
  if(descriptor->GetOption("bubble_band") == "on"){
    m_bubbleBand_handle =
      sfl_create_BubbleBand(m_robotModel_handle,
			    m_odometry_handle,
			    m_multiscanner_handle,
			    params.bband_shortpath,
			    params.bband_longpath,
			    params.bband_maxignoredistance);
    if(0 > m_bubbleBand_handle){
      fprintf(stderr,
	      "ERROR in LAAS ctor: sfl_create_BubbleBand(): %d\n",
	      m_bubbleBand_handle);
      exit(EXIT_FAILURE);
    }
    m_bubbleBand = sfl_cwrap::get_BubbleBand(m_bubbleBand_handle);
    m_motionPlanner_handle =
      expo_create_MotionPlanner(m_motionController_handle,
				m_dynamicWindow_handle,
				m_multiscanner_handle,
				m_robotModel_handle,
				m_bubbleBand_handle,
				m_odometry_handle);
  }
  else if(descriptor->GetOption("bubble_band") == "off"){
    m_bubbleBand_handle = -42;
    m_motionPlanner_handle =
      expo_create_MotionPlanner_nobband(m_motionController_handle,
					m_dynamicWindow_handle,
					m_multiscanner_handle,
					m_robotModel_handle,
					m_odometry_handle);
  }
  else{
    cerr << "invalid bubble_band option \"" <<
      descriptor->GetOption("bubble_band") << "\"\n";
    exit(EXIT_FAILURE);
  }
  if(0 > m_motionPlanner_handle){
    fprintf(stderr,
	    "ERROR in LAAS ctor: sfl_create_MotionPlanner(): %d\n",
	    m_motionPlanner_handle);
    exit(EXIT_FAILURE);
  }
  m_motionPlanner = sfl_cwrap::get_MotionPlanner(m_motionPlanner_handle);
  m_motionPlanner->strict_dwa = params.mp_strict_dwa;
  m_motionPlanner->auto_adapt_dwa = params.mp_auto_adapt_dwa;
  
  CreateGfxStuff(descriptor->name);
}


LAAS::
~LAAS()
{
  expo_destroy_MotionPlanner(m_motionPlanner_handle);
  sfl_destroy_Multiscanner(m_multiscanner_handle);
  if(0 <= m_bubbleBand_handle) sfl_destroy_BubbleBand(m_bubbleBand_handle);
  sfl_destroy_Odometry(m_odometry_handle);
  sfl_destroy_DynamicWindow(m_dynamicWindow_handle);
  expo_destroy_MotionController(m_motionController_handle);
  sfl_destroy_RobotModel(m_robotModel_handle);
  sfl_destroy_Scanner(m_front_handle);
  sfl_destroy_HAL(m_hal_handle);
  if(halmap.find(m_cwrap_hal.get()) != halmap.end()) // paranoid
    halmap.erase(halmap.find(m_cwrap_hal.get()));
}


void LAAS::
CreateGfxStuff(const string & name)
{
  AddDrawing(new MPDrawing(name + "_goaldrawing", *m_motionPlanner));
  AddDrawing(new DWDrawing(name + "_dwdrawing", *m_dynamicWindow));
  AddDrawing(new ODrawing(name + "_dodrawing",
			  m_dynamicWindow->GetDistanceObjective(),
			  *m_dynamicWindow));
  AddDrawing(new ODrawing(name + "_hodrawing",
			  m_dynamicWindow->GetHeadingObjective(),
			  *m_dynamicWindow));
  AddDrawing(new ODrawing(name + "_sodrawing",
			  m_dynamicWindow->GetSpeedObjective(),
			  *m_dynamicWindow));
  if(0 <= m_bubbleBand_handle){
    AddDrawing(new RHDrawing(name + "_rhdrawing",
			     m_bubbleBand->GetReplanHandler(),
			     RHDrawing::AUTODETECT));
    AddDrawing(new BBDrawing(name + "_bbdrawing",
			     *m_bubbleBand,
			     BBDrawing::AUTODETECT));
    AddDrawing(new GridLayerDrawing(name + "_local_gldrawing",
				    m_bubbleBand->GetReplanHandler()->GetNF1(),
				    false));
    AddDrawing(new GridLayerDrawing(name + "_global_gldrawing",
				    m_bubbleBand->GetReplanHandler()->GetNF1(),
				    true));
    AddCamera(new GridLayerCamera(name + "_local_glcamera",
				  m_bubbleBand->GetReplanHandler()->GetNF1()));
  }
  AddDrawing(new OdometryDrawing(name + "_odomdrawing",
				 *m_odometry,
				 m_robotModel->WheelBase() / 2));
  AddDrawing(new DODrawing(name + "_collisiondrawing",
			   m_dynamicWindow->GetDistanceObjective(),
			   *m_dynamicWindow,
			   *m_robotModel));
  
  AddCamera(new StillCamera(name + "_dwcamera",
			    0,
			    0,
			    m_dynamicWindow->Dimension(),
			    m_dynamicWindow->Dimension(),
			    true));
  AddCamera(new OCamera(name + "_ocamera", *m_dynamicWindow));
  double a, b, c, d;
  m_dynamicWindow->GetDistanceObjective().GetRange(a, b, c, d);
  AddCamera(new StillCamera(name + "_collisioncamera", a, b, c, d, true));
}


void LAAS::
InitPose(double x, double y, double theta)
{
  m_odometry->Init(Pose(x, y, theta));
}


void LAAS::
SetPose(double x, double y, double theta)
{
  m_odometry->Set(Pose(x, y, theta));
}


void LAAS::
GetPose(double & x, double & y, double & theta)
{
  shared_ptr<const Pose> pose(m_odometry->Get());
  x = pose->X();
  y = pose->Y();
  theta = pose->Theta();
}


shared_ptr<const Goal> LAAS::
GetGoal()
{
  return shared_ptr<const Goal>(new Goal(m_motionPlanner->GetGoal()));
}


// // bool LAAS::
// // StartThreads()
// // {
// //   shared_ptr<RobotDescriptor> descriptor(GetDescriptor());
// //   if(descriptor->GetOption("front_thread") != ""){
// //     cerr << "front_thread \"" << descriptor->GetOption("front_thread")
// // 	 << "\"\n";
// //     istringstream is(descriptor->GetOption("front_thread"));
// //     unsigned int useconds;
// //     if( ! (is >> useconds))
// //       return false;
// //     shared_ptr<ScannerThread> thread(new ScannerThread("front"));
// //     if( ! m_front->SetThread(thread))
// //       return false;
// //     if( ! thread->Start(useconds))
// //       return false;
// //     cerr << "started front_thread: " << useconds << "us\n";
// //   }
// //   if(descriptor->GetOption("rear_thread") != ""){
// //     cerr << "rear_thread \"" << descriptor->GetOption("rear_thread")
// // 	 << "\"\n";
// //     istringstream is(descriptor->GetOption("rear_thread"));
// //     unsigned int useconds;
// //     if( ! (is >> useconds))
// //       return false;
// //     shared_ptr<ScannerThread> thread(new ScannerThread("rear"));
// //     if( ! m_rear->SetThread(thread))
// //       return false;
// //     if( ! thread->Start(useconds))
// //       return false;
// //     cerr << "started rear_thread: " << useconds << "us\n";
// //   }
// //   if(descriptor->GetOption("mc_thread") != ""){
// //     cerr << "mc_thread \"" << descriptor->GetOption("mc_thread")
// // 	 << "\"\n";
// //     istringstream is(descriptor->GetOption("mc_thread"));
// //     unsigned int useconds;
// //     if( ! (is >> useconds))
// //       return false;
// //     shared_ptr<MotionControllerThread>
// //       thread(new MotionControllerThread("mc"));
// //     if( ! m_motionController->SetThread(thread))
// //       return false;
// //     if( ! thread->Start(useconds))
// //       return false;
// //     cerr << "started mc_thread: " << useconds << "us\n";
// //   }
// //   if(descriptor->GetOption("odo_thread") != ""){
// //     cerr << "odo_thread \"" << descriptor->GetOption("odo_thread")
// // 	 << "\"\n";
// //     istringstream is(descriptor->GetOption("odo_thread"));
// //     unsigned int useconds;
// //     if( ! (is >> useconds))
// //       return false;
// //     shared_ptr<OdometryThread> thread(new OdometryThread("odo"));
// //     if( ! m_odometry->SetThread(thread))
// //       return false;
// //     if( ! thread->Start(useconds))
// //       return false;
// //     cerr << "started odo_thread: " << useconds << "us\n";
// //   }
// //   if(descriptor->GetOption("bband_thread") != ""){
// //     cerr << "bband_thread \"" << descriptor->GetOption("bband_thread")
// // 	 << "\"\n";
// //     istringstream is(descriptor->GetOption("bband_thread"));
// //     unsigned int useconds;
// //     if( ! (is >> useconds))
// //       return false;
// //     shared_ptr<BubbleBandThread> thread(new BubbleBandThread("bband"));
// //     if( ! m_bubbleBand->SetThread(thread))
// //       return false;
// //     if( ! thread->Start(useconds))
// //       return false;
// //     cerr << "started bband_thread: " << useconds << "us\n";
// //   }
// //   return true;  
// // }


bool LAAS::
GoalReached()
{
  const int res(expo_goal_reached(m_motionPlanner_handle));
  if(0 > res){
    cerr << "BUG in LAAS::GoalReached()\n";
    exit(EXIT_FAILURE);
  }
  return res == 1;
}


void LAAS::
SetGoal(double timestep, const Goal & goal)
{
  const int res(expo_set_goal(m_motionPlanner_handle,
			      timestep,
			      goal.X(),
			      goal.Y(),
			      goal.Theta(),
			      goal.Dr(),
			      goal.Dtheta(),
			      goal.IsVia() ? 1 : 0));
  if(0 > res){
    cerr << "BUG in LAAS::SetGoal()\n";
    exit(EXIT_FAILURE);
  }  
}


void LAAS::
PrepareAction(double timestep)
{
  const int res(expo_update_all(m_motionPlanner_handle, timestep));
  if(0 > res){
    cerr << "WARNING in LAAS::DoPrepareAction(): " << res << "\n";
    ////    exit(EXIT_FAILURE);
  }  
}


int cwrap_laas_time_get(struct cwrap_hal_s * self, struct timespec * stamp)
{
  if(halmap.find(self) == halmap.end())
    return -4242;
  return halmap[self]->time_get(stamp);
}


int cwrap_laas_odometry_set(struct cwrap_hal_s * self,
			    double x, double y, double theta,
			    double sxx, double syy, double stt,
			    double sxy, double sxt, double syt)
{
  if(halmap.find(self) == halmap.end())
    return -4242;
  return halmap[self]->odometry_set(x, y, theta,
				    sxx, syy, stt, sxy, sxt, syt);
}


int cwrap_laas_odometry_get(struct cwrap_hal_s * self,
			    struct timespec * stamp,
			    double * x, double * y, double * theta,
			    double * sxx, double * syy, double * stt,
			    double * sxy, double * sxt, double * syt)
{
  if(halmap.find(self) == halmap.end())
    return -4242;
  return halmap[self]->odometry_get(stamp, x, y, theta,
				    sxx, syy, stt, sxy, sxt, syt);
}


int cwrap_laas_speed_set(struct cwrap_hal_s * self, double qdl, double qdr)
{
  if(halmap.find(self) == halmap.end())
    return -4242;
  return halmap[self]->speed_set(qdl, qdr);
}


int cwrap_laas_speed_get(struct cwrap_hal_s * self, double * qdl, double * qdr)
{
  if(halmap.find(self) == halmap.end())
    return -4242;
  return halmap[self]->speed_get(qdl, qdr);
}


int cwrap_laas_scan_get(struct cwrap_hal_s * self,
			int channel, double * rho, size_t * rho_len,
			struct timespec * t0, struct timespec * t1)
{
  if(halmap.find(self) == halmap.end())
    return -4242;
  return halmap[self]->scan_get(channel, rho, rho_len, t0, t1);
}


Jido::
Jido(boost::shared_ptr<npm::RobotDescriptor> descriptor,
     const npm::World & world)
  : LAAS(create_jido_hull(), descriptor, world)
{
}


Rackham::
Rackham(boost::shared_ptr<npm::RobotDescriptor> descriptor,
     const npm::World & world)
  : LAAS(create_rackham_hull(), descriptor, world)
{
}
