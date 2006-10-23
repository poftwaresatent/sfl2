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


#ifndef BIRON_HPP
#define BIRON_HPP


#include <npm/common/RobotClient.hpp>


class Biron
  : public npm::RobotClient
{
private:
  Biron(const Biron &);
  
public:
  Biron(boost::shared_ptr<npm::RobotDescriptor> descriptor,
	const npm::World & world);
  virtual ~Biron();
  
  virtual void PrepareAction(double timestep);
  virtual void InitPose(double x, double y, double theta);
  virtual void SetPose(double x, double y, double theta);
  virtual void GetPose(double & x, double & y, double & theta);
  virtual void SetGoal(double timestep, const sfl::Goal & goal);
  virtual boost::shared_ptr<const sfl::Goal> GetGoal();
  virtual bool GoalReached();
  
protected:
  void LazyXCFInit();
  void LazyXCFCleanup();
  
  boost::shared_ptr<npm::HAL> m_hal;
  boost::shared_ptr<sfl::Scanner> m_sick;
  boost::shared_ptr<sfl::Goal> m_goal;
  bool m_goal_changed;
  int m_nscans, m_sick_channel;
  double m_wheelbase, m_wheelradius;
  int m_speedref_status, m_position_status, m_goal_status,
    m_odometry_status, m_scan_status, m_curspeed_status;
};

#endif // BIRON_HPP
