/* 
 * Copyright (C) 2004
 * Swiss Federal Institute of Technology, Lausanne. All rights reserved.
 * 
 * Developed at the Autonomous Systems Lab.
 * Visit our homepage at http://asl.epfl.ch/
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


#ifndef ROBOX_HPP
#define ROBOX_HPP


#include <npm/common/RobotClient.hpp>


namespace sfl {
  class DynamicWindow;
  class RobotModel;
  class Hull;
  class BubbleBand;
  class Odometry;
  class Scanner;
  class Multiscanner;
}


namespace expo {
  class MotionPlanner;
  class MotionController;
}


class Robox
  : public npm::RobotClient
{
private:
  Robox(const Robox &);
  
protected:
  Robox(boost::shared_ptr<npm::RobotDescriptor> descriptor,
	const npm::World & world);
  
public:
  static Robox *
  Create(boost::shared_ptr<npm::RobotDescriptor> descriptor,
	 const npm::World & world);
  
  virtual bool PrepareAction(double timestep);
  virtual void SetGoal(double timestep, const sfl::Goal & goal);
  virtual bool GoalReached();
  virtual void InitPose(double x, double y, double theta);
  virtual void SetPose(double x, double y, double theta);
  virtual void GetPose(double & x, double & y, double & theta);
  virtual boost::shared_ptr<const sfl::Goal> GetGoal();
  
  static boost::shared_ptr<sfl::Hull> CreateHull();  
  
protected:
  boost::shared_ptr<sfl::Scanner> m_front;
  boost::shared_ptr<sfl::Scanner> m_rear;
  boost::shared_ptr<npm::DiffDrive> m_drive;
  boost::shared_ptr<sfl::RobotModel> m_robotModel;
  boost::shared_ptr<expo::MotionController> m_motionController;
  boost::shared_ptr<sfl::DynamicWindow> m_dynamicWindow;
  boost::shared_ptr<sfl::Odometry> m_odometry;
  boost::shared_ptr<sfl::BubbleBand> m_bubbleBand;
  boost::shared_ptr<sfl::Multiscanner> m_multiscanner;
  boost::shared_ptr<expo::MotionPlanner> m_motionPlanner;
  boost::shared_ptr<sfl::Hull> m_hull;
  
  void CreateGfxStuff(const std::string & name);
  bool StartThreads();
};

#endif // ROBOX_HPP
