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
#include "GenomHAL.hpp"
#include <npm/robox/expoparams.hpp>
#include <npm/common/OdometryDrawing.hpp>
#include <npm/common/StillCamera.hpp>
//#include <npm/common/HAL.hpp>
#include <npm/common/DiffDrive.hpp>
#include <npm/common/RobotDescriptor.hpp>
#include <npm/common/Lidar.hpp>
#include <sfl/util/pdebug.hpp>
#include <sfl/api/RobotModel.hpp>
#include <sfl/api/Goal.hpp>
#include <iostream>
#include <sstream>
#include <map>


#define PDEBUG PDEBUG_OFF


using namespace npm;
using namespace sfl;
using namespace boost;
using namespace std;


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


static RobotModel::Parameters
CreateRobotParameters(expoparams & params)
{
  return RobotModel::
    Parameters(params.model_security_distance,
	       params.model_wheelbase,
	       params.model_wheelradius,
	       params.model_qd_max,
	       params.model_qdd_max,
	       params.model_sd_max,
	       params.model_thetad_max,
	       params.model_sdd_max,
	       params.model_thetadd_max);
}


LAAS::
LAAS(shared_ptr<sfl::Hull> hull,
     shared_ptr<RobotDescriptor> descriptor, const World & world)
  : RobotClient(GenomHALFactory(this), descriptor, world, true), m_hull(hull),
    m_goal(new Goal())
{
  for(size_t ip(0); ip < m_hull->GetNPolygons(); ++ip){
    shared_ptr<const Polygon> poly(m_hull->GetPolygon(ip));
    for(size_t il(0); il < poly->GetNPoints(); ++il)
      AddLine(*poly->_GetLine(il));
  }
  
  expoparams params(descriptor);
  
//     sfl_create_Scanner(m_hal_handle,
// 		       params.front_channel,
// 		       params.front_mount_x,
// 		       params.front_mount_y,
// 		       params.front_mount_theta,
// 		       params.front_nscans,
// 		       params.front_rhomax,
// 		       params.front_phi0,
// 		       params.front_phirange);
//  m_front = sfl_cwrap::get_Scanner(m_front_handle);
//  DefineLidar(m_front);
  
  m_drive = DefineDiffDrive(params.model_wheelbase, params.model_wheelradius);
  
  const RobotModel::Parameters modelParms(CreateRobotParameters(params));
  m_robotModel.reset(new RobotModel(modelParms, m_hull));
  
  CreateGfxStuff(descriptor->name);
}


LAAS::
~LAAS()
{
}


void LAAS::
CreateGfxStuff(const string & name)
{
}


void LAAS::
InitPose(double x, double y, double theta)
{
  GetHAL()->odometry_set(x, y, theta, 1, 1, 1, 0, 0, 0);
}


void LAAS::
SetPose(double x, double y, double theta)
{
  GetHAL()->odometry_set(x, y, theta, 1, 1, 1, 0, 0, 0);
}


void LAAS::
GetPose(double & x, double & y, double & theta)
{
  struct ::timespec stamp;
  double foo;
  if(0 != GetHAL()->odometry_get(&stamp, &x, &y, &theta,
				 &foo, &foo, &foo, &foo, &foo, &foo)){
    x = 0;
    y = 0;
    theta = 0;
  }
}


shared_ptr<const Goal> LAAS::
GetGoal()
{
  return m_goal;
}


bool LAAS::
GoalReached()
{
  double x, y, theta;
  GetPose(x, y, theta);
  const Frame pose(x, y, theta);
  static const bool go_forwards(true);
  return m_goal->Reached(pose, go_forwards);
}


void LAAS::
SetGoal(double timestep, const Goal & goal)
{
  *m_goal = goal;
}


void LAAS::
PrepareAction(double timestep)
{
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
