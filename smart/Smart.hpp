/* -*- mode: C++; tab-width: 2 -*- */
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
#include <sfl/util/vec2d.hpp>
#include <vector>


namespace smart {
  class Algorithm;
	class MappingThread;
	class PlanningThread;
  class ControlThread;
}

namespace asl {
	class ArcControl;
	class AckermannController;
	class NavFuncQuery;
	
  struct path_element;
  typedef std::vector<path_element> path_t;
	typedef sfl::vec2d<double> path_point;
	typedef std::vector<path_point> trajectory_t;
} 


namespace sfl {
  class Multiscanner;
  class Odometry;
  class Scanner;
	class RWlock;
}


class SmartColorScheme;


class Smart
  : public npm::RobotClient
{
private:
  Smart(const Smart &);
  
public:
  Smart(boost::shared_ptr<npm::RobotDescriptor> descriptor,
				const npm::World & world);
	~Smart();
	
  virtual bool PrepareAction(double timestep);
  virtual void InitPose(double x, double y, double theta);
  virtual void SetPose(double x, double y, double theta);
  virtual void GetPose(double & x, double & y, double & theta);
	const sfl::Frame & GetPose() const;
  virtual void SetGoal(double timestep, const sfl::Goal & goal);
  virtual boost::shared_ptr<const sfl::Goal> GetGoal();
  virtual bool GoalReached();
	
	/** \note Can return null. */
	void CopyPaths(boost::shared_ptr<asl::path_t> & clean,
								 boost::shared_ptr<asl::path_t> & dirty) const;
	
	/** \note Can return null. */
	const asl::trajectory_t * GetTrajectory() const;	

	bool GetRefpoint(asl::path_point &ref_point) const;
	
	/** \note Can return null. */
	const asl::ArcControl * GetArcControl() const;
	
	/** \note Can return null. */
	boost::shared_ptr<const asl::NavFuncQuery> GetQuery() const;
	
  
protected:
  boost::shared_ptr<sfl::Scanner> m_sick;
  boost::shared_ptr<sfl::Multiscanner> m_mscan;
  boost::shared_ptr<SmartColorScheme> m_smart_cs;
	boost::shared_ptr<sfl::Odometry> m_odo;
  
  boost::shared_ptr<smart::Algorithm> m_smart_algo;
  boost::shared_ptr<sfl::RWlock> m_simul_rwlock;
  boost::shared_ptr<smart::MappingThread> m_mapping_thread;
  boost::shared_ptr<smart::PlanningThread> m_planning_thread;
  boost::shared_ptr<smart::ControlThread> m_control_thread;
	boost::shared_ptr<const asl::AckermannController> m_acntrl;
	
  int m_nscans, m_sick_channel;
	bool m_error;
	int m_simul_usecsleep;
	int m_mapping_usecsleep;
	int m_planning_usecsleep;
	int m_control_usecsleep;
};

#endif // NPM_SMART_HPP
