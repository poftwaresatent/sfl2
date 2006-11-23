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


#ifndef ESBOT_HPP
#define ESBOT_HPP


#include <npm/common/RobotClient.hpp>
#include <sfl/api/Goal.hpp>
#include <vector>


namespace sfl {
  class DynamicWindow;
  class Odometry;
  class Multiscanner;
  class MotionController;
  class RobotModel;
}


namespace estar {
  struct carrot_item;
  typedef std::vector<carrot_item> carrot_trace;
}


namespace npm {
  class CheatSheet;
  class Lidar;
}


class PNF;


/**
   Mimics most of Robox class, but "simpler" and uses libestar in
   conjunction with libsunflower's DWA implementation.
*/
class Esbot
  : public npm::RobotClient
{
public:
  Esbot(boost::shared_ptr<npm::RobotDescriptor> descriptor,
	const npm::World & world);
  
  virtual void PrepareAction(double timestep);
  virtual void SetGoal(double timestep, const sfl::Goal & goal);
  virtual bool GoalReached();

  virtual void InitPose(double x, double y, double theta);
  virtual void SetPose(double x, double y, double theta);
  virtual void GetPose(double & x, double & y, double & theta);
  virtual boost::shared_ptr<const sfl::Goal> GetGoal();
  
  boost::shared_ptr<PNF> GetPNF() { return m_pnf; }
  boost::shared_ptr<estar::carrot_trace> GetCarrotTrace() const
  { return m_carrot_trace; }
  boost::shared_ptr<estar::carrot_trace> ComputeFullCarrot() const;
  const sfl::Frame & GetPose() const { return *m_pose; }
  
protected:
  const double m_radius;
  const double m_speed;
  const double m_grid_width;
  const size_t m_grid_wdim;
  
  boost::shared_ptr<sfl::Goal> m_goal;
  boost::shared_ptr<sfl::Scanner> m_front;
  boost::shared_ptr<sfl::Scanner> m_rear;
  boost::shared_ptr<npm::DiffDrive> m_drive;
  boost::shared_ptr<sfl::RobotModel> m_robotModel;
  boost::shared_ptr<sfl::MotionController> m_motionController;
  boost::shared_ptr<sfl::DynamicWindow> m_dynamicWindow;
  boost::shared_ptr<sfl::Odometry> m_odometry;
  boost::shared_ptr<sfl::Multiscanner> m_multiscanner;
  boost::shared_ptr<PNF> m_pnf;
  boost::shared_ptr<npm::CheatSheet> m_cheat;
  boost::shared_ptr<estar::carrot_trace> m_carrot_trace;
  boost::shared_ptr<sfl::Frame> m_pose;
  
  bool m_replan_request;
  
  void CreateGfxStuff(const std::string & name);
};

#endif // ESBOT_HPP
