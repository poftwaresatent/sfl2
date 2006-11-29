/* 
 * Copyright (C) 2006
 * Swiss Federal Institute of Technology, Zurich. All rights reserved.
 * 
 * Developed at the Autonomous Systems Lab.
 * Visit our homepage at http://www.asl.ethz.ch/
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

#include "Smart.hpp"
#include <sfl/api/Goal.hpp>
#include <sfl/api/Scanner.hpp>
#include <sfl/util/numeric.hpp>
#include <npm/robox/expoparams.hpp>
#include <npm/common/Lidar.hpp>
#include <npm/common/HAL.hpp>
#include <npm/common/RobotServer.hpp>
#include <npm/common/GoalInstanceDrawing.hpp>
#include <math.h>

using namespace npm;
using namespace sfl;
using namespace boost;
using namespace std;

Smart::
Smart(shared_ptr<RobotDescriptor> descriptor, const World & world)
  : RobotClient(descriptor, world, true),
    m_goal(new Goal()), m_goal_changed(false),
    m_speedref_status(-42), m_position_status(-42), m_goal_status(-42),
    m_odometry_status(-42), m_scan_status(-42), m_curspeed_status(-42)
{

  expoparams params(descriptor);
  m_nscans = params.front_nscans;
  m_sick_channel = params.front_channel;
  m_wheelbase = params.model_wheelbase;
  m_wheelradius = params.model_wheelradius;
  m_axlewidth = params.model_axlewidth;

  m_sick = DefineLidar(Frame(params.front_mount_x,
                             params.front_mount_y,
                             params.front_mount_theta),
                       params.front_nscans,
                       params.front_rhomax,
                       params.front_phi0,
                       params.front_phirange,
                       params.front_channel)->GetScanner();

  DefineBicycleDrive(m_wheelbase, m_wheelradius, m_axlewidth);

  AddLine(Line(0, -m_axlewidth/2,
	       0, m_axlewidth/2));
  AddLine(Line(m_wheelbase, -m_axlewidth/2,
	       m_wheelbase, m_axlewidth/2));
  AddLine(Line(0, -m_axlewidth/2,
	       m_wheelbase, -m_axlewidth/2));
  AddLine(Line(0, m_axlewidth/2,
	       m_wheelbase, m_axlewidth/2));
  
  AddDrawing(new GoalInstanceDrawing(descriptor->name + "_goaldrawing",
				     *m_goal));
}


void Smart::
PrepareAction(double timestep)
{
  m_sick->Update();
  
  const Frame & pose(GetServer()->GetTruePose());
  double dx(m_goal->X() - pose.X());
  double dy(m_goal->Y() - pose.Y());
  pose.RotateFrom(dx, dy);
  const double phimax(M_PI / 2.1);
  double dphi(boundval(-phimax, atan2(dy, dx - m_wheelbase), phimax));
  const double sdmin(0.05);
  const double sdmax(0.8);
  const double scale((phimax - absval(dphi)) / phimax);
  const double sd(sdmax * (1 - scale) + sdmin * scale);
  
  GetHAL()->speed_set(sd, dphi);
}


void Smart::
InitPose(double x, double y, double theta)
{
}


void Smart::
SetPose(double x, double y, double theta)
{
}


void Smart::
GetPose(double & x, double & y, double & theta)
{
  const Frame & pose(GetServer()->GetTruePose());
  x = pose.X();
  y = pose.Y();
  theta = pose.Theta();
}


void Smart::
SetGoal(double timestep, const sfl::Goal & goal)
{
  *m_goal = goal;
}


shared_ptr<const Goal> Smart::
GetGoal()
{
  return m_goal;
}


bool Smart::
GoalReached()
{
  return m_goal->DistanceReached(GetServer()->GetTruePose());
}

