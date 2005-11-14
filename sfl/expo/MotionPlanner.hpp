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


#include <sfl/api/Multiscanner.hpp>
#include <sfl/api/MotionPlanner.hpp>
#include <sfl/expo/MotionPlannerFields.hpp>
#include <sfl/expo/MotionPlannerState.hpp>


namespace expo {


  class MotionPlanner:
    public sfl::MotionPlanner
  {
  public:
    enum state_id_t {
      take_aim,
      aimed,
      adjust_goal_heading,
      at_goal,
      null
    };
    
    
    MotionPlanner(MotionController & motion_controller,
		  sfl::DynamicWindow & dynamic_window,
		  sfl::Multiscanner & multiscanner,
		  const sfl::RobotModel & robot_model,
		  sfl::BubbleBand & bubble_band,
		  const sfl::Odometry & odometry);

    void Update();
    void SetGoal(const sfl::Goal & goal);
    const sfl::Goal & GetGoal() const;
    bool GoalReached() const;
    state_id_t GetStateId() const;
    const char * GetStateName() const;
    
    /** \note Hack for Cogniron, does ugly things like const_casts!
	\return <ul><li>  0: success                        </li>
                    <li> -1: odometry update error          </li>
		    <li> -2: multiscanner update error      </li>
		    <li> -3: motion controller update error </li></ul> */
    int UpdateAll();
    
  private:
    MotionPlannerFields _fields;
    MotionPlannerState * _internal_state;
    sfl::Multiscanner & _multiscanner;
  };

}

#endif // EXPO_MOTIONPLANNER_HPP
