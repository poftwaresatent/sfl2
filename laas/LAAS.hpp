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


#ifndef LAAS_HPP
#define LAAS_HPP


#include <npm/common/RobotClient.hpp>
#include <boost/scoped_ptr.hpp>


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


class LAAS
  : public npm::RobotClient
{
private:
  LAAS(const LAAS &);
  
protected:
  LAAS(boost::shared_ptr<sfl::Hull> hull,
       boost::shared_ptr<npm::RobotDescriptor> descriptor,
       const npm::World & world);
  
public:
  ~LAAS();
  
  virtual void PrepareAction(double timestep);
  virtual void InitPose(double x, double y, double theta);
  virtual void SetPose(double x, double y, double theta);
  virtual void GetPose(double & x, double & y, double & theta);
  virtual boost::shared_ptr<const sfl::Goal> GetGoal();
  virtual bool GoalReached();
  virtual void SetGoal(double timestep, const sfl::Goal & goal);
  
protected:
  void CreateGfxStuff(const std::string & name);
  
  boost::shared_ptr<sfl::Scanner> m_front;
  boost::shared_ptr<npm::DiffDrive> m_drive;
  boost::shared_ptr<sfl::RobotModel> m_robotModel;
  boost::shared_ptr<expo::MotionController> m_motionController;
  boost::shared_ptr<sfl::DynamicWindow> m_dynamicWindow;
  boost::shared_ptr<sfl::Odometry> m_odometry;
  boost::shared_ptr<sfl::BubbleBand> m_bubbleBand;
  boost::shared_ptr<sfl::Multiscanner> m_multiscanner;
  boost::shared_ptr<expo::MotionPlanner> m_motionPlanner;
  boost::shared_ptr<sfl::Hull> m_hull;
  
  boost::scoped_ptr<struct cwrap_hal_s> m_cwrap_hal;
  
  int m_hal_handle;
  int m_front_handle;
  int m_robotModel_handle;
  int m_motionController_handle;
  int m_dynamicWindow_handle;
  int m_odometry_handle;
  int m_bubbleBand_handle;
  int m_multiscanner_handle;
  int m_motionPlanner_handle;
};


class Jido:
  public LAAS
{
public:
  Jido(boost::shared_ptr<npm::RobotDescriptor> descriptor,
       const npm::World & world);
};


class Rackham:
  public LAAS
{
public:
  Rackham(boost::shared_ptr<npm::RobotDescriptor> descriptor,
	  const npm::World & world);
};

#endif // LAAS_HPP
