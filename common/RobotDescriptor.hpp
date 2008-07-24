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


#ifndef NPM_ROBOTDESCRIPTOR_HPP
#define NPM_ROBOTDESCRIPTOR_HPP


#include <sfl/util/OptionDictionary.hpp>
#include <boost/shared_ptr.hpp>
#include <string>
#include <map>
#include <vector>


namespace sfl {
  class Goal;
  class Frame;
}


namespace npm {
  
  
  /**
     Mainly for lazy initialization of Robot (sub)class instances, but
     also for managing goals and options (which might have to be
     available for subclass ctor calls, passing a Descriptor instance
     makes that possible).
  */
  class RobotDescriptor
    : public sfl::OptionDictionary
  {
  public:
    RobotDescriptor(const std::string & model, const std::string & name);
    
    void SetInitialPose(double x, double y, double theta);
    boost::shared_ptr<const sfl::Frame> GetInitialPose() const
    { return m_initial_pose; }
    
    bool HaveGoals() const { return ! m_goal.empty(); }
    
    /** \return 0 iff m_goal.empty(), a valid pointer otherwise. */
    sfl::Goal * GetCurrentGoal();
    
    void AddGoal(double x, double y, double theta, double dr, double dtheta);
    
    /** Like AddGoal(), but after NextGoal() has reached this one, it
	won't change (loop) afterwards. */
    void AddEndGoal(double x, double y, double theta, double dr,
		    double dtheta);
    
    /** Used to cycle through the goals registered with AddGoal(). */
    void NextGoal();
    
    const std::string model;
    const std::string name;
    
  private:
    typedef std::vector<boost::shared_ptr<sfl::Goal> > goal_t;
    
    boost::shared_ptr<sfl::Frame> m_initial_pose;
    goal_t m_goal;
    ssize_t m_current_goal;
    ssize_t m_stop_loop_idx;
  };

}

#endif // NPM_ROBOTDESCRIPTOR_HPP
