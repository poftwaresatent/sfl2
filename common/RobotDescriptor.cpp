/* 
 * Copyright (C) 2005
 * Centre National de Recherche Scientifique, France.
 * All rights reserved.
 * 
 * Developed at
 * Laboratoire d'Automatique et d'Analyse des Systemes, LAAS-CNRS.
 * Visit our homepage at http://www.laas.fr/
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


#include "RobotDescriptor.hpp"
#include <sfl/api/Goal.hpp>

using namespace sfl;
using namespace boost;
using namespace std;


namespace npm {


  RobotDescriptor::
  RobotDescriptor(const string & _model, const string & _name)
    : model(_model), name(_name), m_current_goal(0), m_stop_loop_idx(-1)
  {
  }
  
  
  void RobotDescriptor::
  SetInitialPose(double x, double y, double theta)
  {
    if( ! m_initial_pose)
      m_initial_pose.reset(new Frame(x, y, theta));
    else
      m_initial_pose->Set(x, y, theta);
  }


  Goal * RobotDescriptor::
  GetCurrentGoal()
  {
    if(m_goal.empty())
      return 0;
    return m_goal[m_current_goal].get();
  }


  void RobotDescriptor::
  AddGoal(double x, double y, double theta, double dr, double dtheta)
  {
    if(m_stop_loop_idx >= 0)
      return;
    if(m_goal.empty())
      m_current_goal = 0;		// paranoid redundancy
    m_goal.push_back(shared_ptr<Goal>(new Goal(x, y, theta, dr, dtheta,
					       true)));
  }


  void RobotDescriptor::
  AddEndGoal(double x, double y, double theta, double dr, double dtheta)
  {
    if(m_stop_loop_idx >= 0)
      return;
    if(m_goal.empty())
      m_current_goal = 0;		// paranoid redundancy
    m_goal.push_back(shared_ptr<Goal>(new Goal(x, y, theta, dr, dtheta,
					       false)));
    m_stop_loop_idx = m_goal.size() - 1;
  }


  void RobotDescriptor::
  NextGoal()
  {
    if(m_stop_loop_idx == m_current_goal)
      return;
    ++m_current_goal;
    if(m_goal.size() <= static_cast<size_t>(m_current_goal))
      m_current_goal = 0;
  }
  
  
  void RobotDescriptor::
  AddCustomLine(std::string const & line)
  {
    m_custom_lines.push_back(line);
  }
  
  
  std::vector<std::string> const & RobotDescriptor::
  GetCustomLines() const
  {
    return m_custom_lines;
  }
  
}
