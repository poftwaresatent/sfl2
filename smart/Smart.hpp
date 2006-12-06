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

#ifndef NPM_SMART_HPP
#define NPM_SMART_HPP


#include <npm/common/RobotClient.hpp>


namespace npm {
  class CheatSheet;
}


namespace sfl {
  class GridFrame;
  class TraversabilityMap;
}


namespace estar {
  class Facade;
  class Region;
}


class PlanThread;


class Smart
  : public npm::RobotClient
{
private:
  Smart(const Smart &);
  
public:
  Smart(boost::shared_ptr<npm::RobotDescriptor> descriptor,
	const npm::World & world);
  
  virtual void PrepareAction(double timestep);
  virtual void InitPose(double x, double y, double theta);
  virtual void SetPose(double x, double y, double theta);
  virtual void GetPose(double & x, double & y, double & theta);
  virtual void SetGoal(double timestep, const sfl::Goal & goal);
  virtual boost::shared_ptr<const sfl::Goal> GetGoal();
  virtual bool GoalReached();

protected:
  friend class SmartPlanProxy;
  
  boost::shared_ptr<npm::HAL> m_hal;
  boost::shared_ptr<sfl::Scanner> m_sick;
  boost::shared_ptr<sfl::Goal> m_goal;
  boost::shared_ptr<estar::Region> m_goalregion;
  boost::shared_ptr<estar::Facade> m_estar;
  boost::shared_ptr<npm::CheatSheet> m_cheat;
  boost::shared_ptr<PlanThread> m_plan_thread;
  boost::shared_ptr<const sfl::GridFrame> m_gframe;
  boost::shared_ptr<const sfl::TraversabilityMap> m_travmap;
  
  bool m_replan_request;
  int m_nscans, m_sick_channel;
  double m_wheelbase, m_wheelradius, m_axlewidth;
};

#endif // NPM_SMART_HPP
