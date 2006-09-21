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


#ifndef EXPO_MOTIONPLANNER_HPP
#define EXPO_MOTIONPLANNER_HPP


#include <sfl/api/MotionPlanner.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/scoped_ptr.hpp>


namespace sfl {
  class DynamicWindow;
  class Multiscanner;
  class RobotModel;
  class BubbleBand;
  class Odometry;
  class Multiscanner;
}


namespace expo {
  
  
  class MotionPlannerState;
  class MotionController;
  
  
  /** \note lots of public fields for historical reasons... */
  class MotionPlanner:
    public sfl::MotionPlanner
  {
  public:
    enum state_id_t { take_aim, aimed, adjust_goal_heading, at_goal, null };
    
    MotionPlanner(boost::shared_ptr<MotionController> motion_controller,
		  boost::shared_ptr<sfl::DynamicWindow> dynamic_window,
		  boost::shared_ptr<sfl::Multiscanner> multiscanner,
		  boost::shared_ptr<const sfl::RobotModel> robot_model,
		  /** optional: if null then go straight towards goal */
		  boost::shared_ptr<sfl::BubbleBand> bubble_band,
		  boost::shared_ptr<const sfl::Odometry> odometry);
    
    void Update(double timestep);
    void SetGoal(const sfl::Goal & goal);
    const sfl::Goal & GetGoal() const;
    bool GoalReached() const;
    state_id_t GetStateId() const;
    const char * GetStateName() const;
    void GoForward();
    void GoBackward();
    
    /** \note Hack for Cogniron, does ugly things like const_casts!
	\return <ul><li>  0: success                        </li>
                    <li> -1: odometry update error          </li>
		    <li> -2: multiscanner update error      </li>
		    <li> -3: motion controller update error </li></ul> */
    int UpdateAll(double timestep);
    
    boost::shared_ptr<MotionController> motion_controller;
    boost::shared_ptr<sfl::DynamicWindow> dynamic_window;
    boost::shared_ptr<const sfl::RobotModel> robot_model;
    boost::shared_ptr<sfl::BubbleBand> bubble_band; // can be null!
    boost::shared_ptr<const sfl::Odometry> odometry;
    boost::shared_ptr<sfl::Multiscanner> multiscanner;
    
    boost::scoped_ptr<MotionPlannerState> null_state;
    boost::scoped_ptr<MotionPlannerState> take_aim_state;
    boost::scoped_ptr<MotionPlannerState> aimed_state;
    boost::scoped_ptr<MotionPlannerState> adjust_goal_heading_state;
    boost::scoped_ptr<MotionPlannerState> at_goal_state;
    
    boost::scoped_ptr<sfl::Goal> goal;
    bool go_forward, strict_dwa;
    
  private:
    MotionPlannerState * m_internal_state;
    bool m_replan_request;
  };

}

#endif // EXPO_MOTIONPLANNER_HPP
