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
#include "MotionController.hpp"
#include "MotionPlannerState.hpp"
#include <sfl/api/Multiscanner.hpp>
#include <sfl/api/Goal.hpp>
#include <sfl/api/Pose.hpp>
#include <sfl/api/Odometry.hpp>
#include <sfl/bband/BubbleBand.hpp>
#include <sfl/dwa/DynamicWindow.hpp>


using namespace boost;


namespace expo {
  
  
  MotionPlanner::
  MotionPlanner(shared_ptr<MotionController> _motion_controller,
		shared_ptr<sfl::DynamicWindow> _dynamic_window,
		shared_ptr<sfl::Multiscanner> _multiscanner,
		shared_ptr<const sfl::RobotModel> _robot_model,
		shared_ptr<sfl::BubbleBand> _bubble_band,
		shared_ptr<const sfl::Odometry> _odometry)
    : motion_controller(_motion_controller),
      dynamic_window(_dynamic_window),
      robot_model(_robot_model),
      bubble_band(_bubble_band),
      odometry(_odometry),
      multiscanner(_multiscanner),
      null_state(new NullState(this)),
      take_aim_state(new TakeAimState(this)),
      aimed_state(new AimedState(this)),
      adjust_goal_heading_state(new AdjustGoalHeadingState(this)),
      at_goal_state(new AtGoalState(this)),
      goal(new sfl::Goal()),
      go_forward(true),
      strict_dwa(true),
      m_internal_state(null_state.get()),
      m_replan_request(false)
  {
  }
  
  
  void MotionPlanner::
  Update(double timestep)
  {
    shared_ptr<const sfl::Pose> pose(odometry->Get());
    m_internal_state = m_internal_state->NextState(timestep);
    m_internal_state->Act(timestep, multiscanner->CollectScans());
  }
  
  
  void MotionPlanner::
  SetGoal(const sfl::Goal & _goal)
  {
    goal->Set(_goal);
    if(bubble_band)
      bubble_band->SetGoal(_goal);
    m_internal_state = m_internal_state->GoalChangedState();
  }
  
  
  const sfl::Goal & MotionPlanner::
  GetGoal() const
  {
    return * goal;
  }
  
  
  bool MotionPlanner::
  GoalReached() const
  {
    return m_internal_state->GoalReached();
  }
  
  
  int MotionPlanner::
  UpdateAll(double timestep)
  {
    sfl::Odometry & odo(const_cast<sfl::Odometry &>(*odometry));
    if(0 > odo.Update())
      return -1;
    if( ! multiscanner->UpdateAll())
      return -2;
    Update(timestep);
    if(0 > motion_controller->Update(timestep))
      return -3;
    return 0;
  }
  
  
  MotionPlanner::state_id_t MotionPlanner::
  GetStateId() const
  {
    if(m_internal_state == take_aim_state.get())
      return take_aim;
    if(m_internal_state == at_goal_state.get())
      return at_goal;
    if(m_internal_state == aimed_state.get())
      return aimed;
    if(m_internal_state == adjust_goal_heading_state.get())
      return adjust_goal_heading;
    if(m_internal_state == at_goal_state.get())
      return at_goal;
    if(m_internal_state == null_state.get())
      return null;
    return null;
  }
  
  
  const char * MotionPlanner::
  GetStateName() const
  {
    if(m_internal_state == null_state.get())
      return "NULL";
    if(m_internal_state == take_aim_state.get())
      return "TAKE_AIM";
    if(m_internal_state == aimed_state.get())
      return "AIMED";
    if(m_internal_state == adjust_goal_heading_state.get())
      return "ADJUST_GOAL_HEADING";
    if(m_internal_state == at_goal_state.get())
      return "AT_GOAL";
    return "<invalid>";
  }
  
  
  void MotionPlanner::
  GoForward()
  {
    m_internal_state->GoForward(true);
    dynamic_window->GoForward();
  }
  
  
  void MotionPlanner::
  GoBackward()
  {
    m_internal_state->GoForward(false);
    dynamic_window->GoBackward();
  }
  
}
