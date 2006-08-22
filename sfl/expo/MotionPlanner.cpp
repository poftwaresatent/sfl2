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


#include "MotionPlanner.hpp"
#include "MotionPlannerState.hpp"
#include <sfl/api/Multiscanner.hpp>
#include <sfl/api/Pose.hpp>


using namespace boost;


namespace expo {
  
  
  MotionPlanner::
  MotionPlanner(MotionController & motion_controller,
		sfl::DynamicWindow & dynamic_window,
		sfl::Multiscanner & multiscanner,
		const sfl::RobotModel & robot_model,
		sfl::BubbleBand & bubble_band,
		const sfl::Odometry & odometry):
    _fields(motion_controller,
	    dynamic_window,
	    robot_model,
	    bubble_band,
	    odometry),
    _multiscanner(multiscanner)
  {
    _internal_state = _fields.null_state;
  }
  
  
  void MotionPlanner::
  Update(double timestep)
  {
    shared_ptr<const sfl::Pose> pose(_fields.odometry.Get());
    _internal_state = _internal_state->NextState(timestep);
    _internal_state->Act(timestep, _multiscanner.CollectGlobalScans( * pose));
  }
  
  
  void MotionPlanner::
  SetGoal(const sfl::Goal & goal)
  {
    _fields.goal.Set(goal);
    _fields.bubbleBand.SetGoal(goal);
    _internal_state = _internal_state->GoalChangedState();
  }
  
  
  const sfl::Goal & MotionPlanner::
  GetGoal()
    const
  {
    return _fields.goal;
  }
  
  
  bool MotionPlanner::
  GoalReached()
    const
  {
    return _internal_state->GoalReached();
  }
  
  
  int MotionPlanner::
  UpdateAll(double timestep)
  {
    sfl::Odometry & odo(const_cast<sfl::Odometry &>(_fields.odometry));
    if(0 > odo.Update())
      return -1;
    if(0 > _multiscanner.UpdateAll())
      return -2;
    Update(timestep);
    if(0 > _fields.motionController.Update(timestep))
      return -3;
    return 0;
  }
  
  
  MotionPlanner::state_id_t MotionPlanner::
  GetStateId()
    const
  {
    if(_internal_state == _fields.take_aim_state)
      return take_aim;
    if(_internal_state == _fields.at_goal_state)
      return at_goal;
    if(_internal_state == _fields.aimed_state)
      return aimed;
    if(_internal_state == _fields.adjust_goal_heading_state)
      return adjust_goal_heading;
    if(_internal_state == _fields.at_goal_state)
      return at_goal;
    if(_internal_state == _fields.null_state)
      return null;
    return null;
  }
  
  
  const char * MotionPlanner::
  GetStateName()
    const
  {
    if(_internal_state == _fields.null_state)
      return "NULL";
    if(_internal_state == _fields.take_aim_state)
      return "TAKE_AIM";
    if(_internal_state == _fields.aimed_state)
      return "AIMED";
    if(_internal_state == _fields.adjust_goal_heading_state)
      return "ADJUST_GOAL_HEADING";
    if(_internal_state == _fields.at_goal_state)
      return "AT_GOAL";
    return "<invalid>";
  }
  
  
  void MotionPlanner::
  GoForward()
  {
    _internal_state->GoForward(true);
    _fields.dynamicWindow.GoForward();
  }
  
  
  void MotionPlanner::
  GoBackward()
  {
    _internal_state->GoForward(false);
    _fields.dynamicWindow.GoBackward();
  }
  
}
