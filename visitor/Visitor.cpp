/* 
 * Copyright (C) 2004
 * Swiss Federal Institute of Technology, Lausanne. All rights reserved.
 * 
 * Author: Roland Philippsen <roland dot philippsen at gmx dot net>
 *         Autonomous Systems Lab <http://asl.epfl.ch/>
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


#include "Visitor.hpp"
#include <npm/common/HAL.hpp>
#include <npm/common/HoloDrive.hpp>
#include <npm/common/RobotServer.hpp>
#include <sfl/util/Line.hpp>
#include <sfl/util/numeric.hpp>
#include <sfl/api/Goal.hpp>


using namespace npm;
using namespace sfl;
using namespace boost;


Visitor::
Visitor(shared_ptr<RobotDescriptor> descriptor,
	const World & world)
  : RobotClient(descriptor, world, false),
    m_goal(new Goal())
{
  m_drive = DefineHoloDrive(0.6);
  
  static const double foo(0.15);
  static const double bar(0.30);
  
  AddLine(Line(-foo, -bar, -foo,  bar));
  AddLine(Line(-foo,  bar,  foo,  bar));
  AddLine(Line( foo,  bar,  foo, -bar));
  AddLine(Line( foo, -bar, -foo, -bar));
}


void Visitor::
PrepareAction(double timestep)
{
  static const double dthetathresh(5 * M_PI / 180);
  static const double thetadmax(0.8 * M_PI);
  static const double sdmax(0.4);
  
  const Frame & pose(GetServer()->GetTruePose());
  double dx(m_goal->X() - pose.X());
  double dy(m_goal->Y() - pose.Y());
  pose.RotateFrom(dx, dy);
  double dtheta(atan2(dy, dx));
  
  double thetad(dtheta / timestep);
  double xd(sqrt(dx * dx + dy * dy) / timestep);
  double yd(0);

  if(absval(dtheta) > dthetathresh)
    dx = 0;
  
  GetHAL()->speed_set(boundval(-sdmax, xd, sdmax),
		      boundval(-sdmax, yd, sdmax),
		      boundval(-thetadmax, thetad, thetadmax));
}


void Visitor::
InitPose(double x, double y, double theta)
{
}


void Visitor::
SetPose(double x, double y, double theta)
{
}



void Visitor::
GetPose(double &x, double &y, double &theta)
{
  const Frame & pose(GetServer()->GetTruePose());
  x = pose.X();
  y = pose.Y();
  theta = pose.Theta();
}


void Visitor::
SetGoal(double timestep, const Goal & goal)
{
  *m_goal = goal;
}


shared_ptr<const Goal> Visitor::
GetGoal()
{
  return m_goal;
}


bool Visitor::
GoalReached()
{
  return m_goal->DistanceReached(GetServer()->GetTruePose());
}
