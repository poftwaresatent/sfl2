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
  {
  public:
    RobotDescriptor(const std::string & model, const std::string & name);
    
    /** \note returns empty string if undefined name */
    std::string GetOption(const std::string & key) const;
    
    /** \note overrides already existing values */
    void SetOption(const std::string & key, const std::string & value);
    
    void SetInitialPose(double x, double y, double theta);
    boost::shared_ptr<const sfl::Frame> GetInitialPose() const
    { return m_initial_pose; }
    
    bool HaveGoals() const { return ! m_goal.empty(); }
    
    /** \return 0 iff m_goal.empty(), a valid pointer otherwise. */
    sfl::Goal * GetCurrentGoal();
    
    void AddGoal(double x, double y, double theta, double dr, double dtheta);
    
    /** Used to cycle through the goals registered with AddGoal(). */
    void NextGoal();
    
    const std::string model;
    const std::string name;
    
  private:
    typedef std::map<std::string, std::string> option_t;
    typedef std::vector<boost::shared_ptr<sfl::Goal> > goal_t;
    
    boost::shared_ptr<sfl::Frame> m_initial_pose;
    option_t m_option;
    goal_t m_goal;
    size_t m_current_goal;
  };

}

#endif // NPM_ROBOTDESCRIPTOR_HPP
