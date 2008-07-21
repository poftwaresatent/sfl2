/* 
 * rosnpm
 * Copyright (C) 2008, Willow Garage, Inc.
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

#ifndef ROSBOT_HPP
#define ROSBOT_HPP

#include <npm/common/RobotClient.hpp>
#include <vector>


namespace sfl {
  class MotionController;
  class RobotModel;
  class Hull;
}

class ROSbotNode;


class ROSbot
  : public npm::RobotClient
{
public:
  ROSbot(boost::shared_ptr<npm::RobotDescriptor> descriptor,
	 const npm::World & world);
  virtual ~ROSbot();
  
  virtual bool PrepareAction(double timestep);
  virtual void SetGoal(double timestep, const sfl::Goal & goal);
  virtual bool GoalReached();
  virtual void InitPose(double x, double y, double theta);
  virtual void SetPose(double x, double y, double theta);
  virtual void GetPose(double & x, double & y, double & theta);
  virtual boost::shared_ptr<const sfl::Goal> GetGoal();
  
protected:
  friend class ROSbotNode;
  
  std::vector<boost::shared_ptr<npm::Lidar> > m_lidar;
  boost::shared_ptr<npm::DiffDrive> m_drive;
  boost::shared_ptr<sfl::Goal> m_goal;
  boost::shared_ptr<sfl::MotionController> m_mcontrol;
  boost::shared_ptr<sfl::RobotModel> m_model;
  boost::shared_ptr<sfl::Hull> m_hull;
  ROSbotNode * m_ros_node;
};

#endif // ROSBOT_HPP
