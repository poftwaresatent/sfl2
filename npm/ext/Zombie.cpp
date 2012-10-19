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

#include "Zombie.hpp"
#include <npm/HAL.hpp>
#include <npm/HoloDrive.hpp>
#include <npm/RobotServer.hpp>
#include <npm/Lidar.hpp>
#include <sfl/util/Line.hpp>
#include <sfl/util/numeric.hpp>
#include <sfl/api/Goal.hpp>
#include <sfl/api/Scanner.hpp>

using namespace sfl;
using namespace boost;

namespace npm {
  
  
  Zombie::
  Zombie(shared_ptr<RobotDescriptor> descriptor,
	 const World & world)
    : RobotClient(descriptor, world, 3, false),
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
  
  
  bool Zombie::
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
    
    double qd[3] = {
      boundval(-sdmax, xd, sdmax),
      boundval(-sdmax, yd, sdmax),
      boundval(-thetadmax, thetad, thetadmax)
    };
    size_t len(3);
    
    return (0 == GetHAL()->speed_set(qd, &len)) && (3 == len);
  }
  
  
  void Zombie::
  InitPose(double x, double y, double theta)
  {
  }
  
  
  void Zombie::
  SetPose(double x, double y, double theta)
  {
  }
  
  
  void Zombie::
  GetPose(double &x, double &y, double &theta)
  {
    const Frame & pose(GetServer()->GetTruePose());
    x = pose.X();
    y = pose.Y();
    theta = pose.Theta();
  }
  
  
  void Zombie::
  SetGoal(double timestep, const Goal & goal)
  {
    *m_goal = goal;
  }
  
  
  shared_ptr<const Goal> Zombie::
  GetGoal()
  {
    return m_goal;
  }
  
  
  bool Zombie::
  GoalReached()
  {
    return m_goal->DistanceReached(GetServer()->GetTruePose());
  }
  
  
  LidarZombie::
  LidarZombie(shared_ptr<RobotDescriptor> descriptor,
	      const World & world)
    : Zombie(descriptor, world)
  {
    m_scanner = DefineLidar(Frame(0.0, 0.0, 0.0),
			    361, 8.0, -M_PI/2, M_PI,
			    0)->GetScanner();
  }
  
  
  bool LidarZombie::
  PrepareAction(double timestep)
  {
    m_scanner->Update();
    return Zombie::PrepareAction(timestep);
  }
  
}
