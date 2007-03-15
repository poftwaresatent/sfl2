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
#include <boost/scoped_ptr.hpp>
#include <vector>


namespace npm {
  class CheatSheet;
}


namespace sfl {
  class TraversabilityMap;
  class Mapper2d;
  class Frame;
  class GridFrame;
  class Multiscanner;
  class Scanner;
  class Scan;
}


namespace estar {
  class Facade;
  class Region;
  struct carrot_item;
  typedef std::vector<carrot_item> carrot_trace;
}


namespace asl {
  class AckermannController;
  struct path_element;
  typedef std::vector<path_element> path_t;
}


class PlanThread;
class SmartCarrotProxy;
class SmartColorScheme;
class SmartDrawCallback;


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
  
  /** default false */
  bool single_step_estar;
  bool finish_estar;
  
protected:
  bool HandleReplanRequest(const sfl::GridFrame & gframe);
  void UpdatePlan(const sfl::Frame & pose, const sfl::GridFrame & gframe,
		  const sfl::Scan & scan, bool replan);
  bool ComputePath(const sfl::Frame & pose, const sfl::GridFrame & gframe,
		   asl::path_t & path);

  friend class SmartPlanProxy;
  friend class SmartNavFuncQuery;
  friend class SmartCarrotProxy;
  
  boost::shared_ptr<sfl::Scanner> m_sick;
  boost::shared_ptr<sfl::Multiscanner> m_mscan;
  boost::shared_ptr<sfl::Goal> m_goal;
  boost::shared_ptr<estar::Region> m_goalregion;
  boost::shared_ptr<estar::Facade> m_estar;
  boost::shared_ptr<npm::CheatSheet> m_cheat;
  boost::shared_ptr<PlanThread> m_plan_thread;
  boost::shared_ptr<asl::AckermannController> m_controller;
  boost::shared_ptr<SmartCarrotProxy> m_carrot_proxy;
  boost::shared_ptr<estar::carrot_trace> m_carrot_trace;
  boost::shared_ptr<sfl::Mapper2d> m_mapper;
  boost::shared_ptr<const sfl::TraversabilityMap> m_travmap;
  boost::shared_ptr<sfl::Frame> m_last_plan_pose;
  boost::scoped_ptr<SmartColorScheme> m_smart_cs;
  boost::scoped_ptr<SmartDrawCallback> m_cb;
  
  bool m_replan_request;
  int m_nscans, m_sick_channel;
  double m_wheelbase, m_wheelradius, m_axlewidth;
  int m_plan_status;
  double m_replan_distance;
  bool m_discover_travmap;
  double m_carrot_distance, m_carrot_stepsize;
  size_t m_carrot_maxnsteps;
};

#endif // NPM_SMART_HPP
