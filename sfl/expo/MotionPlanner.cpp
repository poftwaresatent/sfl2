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
using namespace std;


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
  Update()
  {
    _internal_state = _internal_state->NextState();
    _internal_state->Act(_multiscanner.
			 CollectGlobalScans(_fields.odometry.Get()));
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
  UpdateAll()
  {
    sfl::Odometry & odo(const_cast<sfl::Odometry &>(_fields.odometry));
    if(0 > odo.Update())
      return -1;
    if(0 > _multiscanner.UpdateAll())
      return -2;
    Update();
    return _fields.motionController.Update();
  }
  
  
  //  void MotionPlanner::
  //  GoForward(){
  //    _internal_state->GoForward(true);
  //    _fields.dynamicWindow.GoForward();
  //  }



  //  void MotionPlanner::
  //  GoBackward(){
  //    _internal_state->GoForward(false);
  //    _fields.dynamicWindow.GoBackward();
  //  }
}
