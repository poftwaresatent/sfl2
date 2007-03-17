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
#include <boost/scoped_ptr.hpp>
#include <vector>


namespace npm {
  class CheatSheet;
}


namespace asl {
  class SmartAlgo;

  struct path_element;
  typedef std::vector<path_element> path_t;
}


namespace sfl {
  class TraversabilityMap;
  class Multiscanner;
  class Scanner;
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
  
  virtual void PrepareAction(double timestep);
  virtual void InitPose(double x, double y, double theta);
  virtual void SetPose(double x, double y, double theta);
  virtual void GetPose(double & x, double & y, double & theta);
  virtual void SetGoal(double timestep, const sfl::Goal & goal);
  virtual boost::shared_ptr<const sfl::Goal> GetGoal();
  virtual bool GoalReached();
	
	const asl::path_t * GetPath() const;
	
  
protected:
  boost::shared_ptr<sfl::Scanner> m_sick;
  boost::shared_ptr<sfl::Multiscanner> m_mscan;
  boost::shared_ptr<npm::CheatSheet> m_cheat;
  boost::scoped_ptr<SmartColorScheme> m_smart_cs;
  
  boost::scoped_ptr<asl::SmartAlgo> m_smart_algo;
  boost::shared_ptr<const sfl::TraversabilityMap> m_cheat_travmap;
  
  int m_nscans, m_sick_channel;
  bool m_discover_travmap;
};

#endif // NPM_SMART_HPP
