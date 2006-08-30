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


#include "MotionPlannerState.hpp"
#include <sfl/api/Pose.hpp>
#include <sfl/bband/BubbleBand.hpp>
#include <cmath>


using namespace boost;
using namespace std;


namespace expo {


  const double TakeAimState::DTHETASTARTHOMING(10 * M_PI / 180);
  const double AimedState::DTHETASTARTAIMING(45 * M_PI / 180);


  MotionPlannerState::
  MotionPlannerState(const string & name,
		     MotionPlannerFields * fields):
    _fields(fields),
    _name(name)
  {
  }


  MotionPlannerState::
  ~MotionPlannerState()
  {
  }


  const string & MotionPlannerState::
  Name()
    const
  {
    return _name;
  }


  bool MotionPlannerState::
  GoalReached()
    const
  {
    shared_ptr<const sfl::Pose> pose(_fields->odometry.Get());
    return ( ! _fields->motionController.Moving())
      && _fields->goal.Reached( * pose, true);
  }


  void MotionPlannerState::
  GoForward(bool b)
  {
    _fields->goForward = b;
  }


  MotionPlannerState * MotionPlannerState::
  NextState(double timestep)
  {
    shared_ptr<const sfl::Pose> pose(_fields->odometry.Get());
    if(_fields->goal.DistanceReached( * pose)){
      double dheading;
      if(_fields->goal.HeadingReached( * pose, _fields->goForward, dheading)
	 && _fields->motionController.Stoppable(timestep)){
	return _fields->at_goal_state;
      }
    
      return _fields->adjust_goal_heading_state;      
    }
 
    return this;
  }


  MotionPlannerState * MotionPlannerState::
  FollowTargetState()
  {
    return _fields->aimed_state;
  }


  MotionPlannerState * MotionPlannerState::
  GoalChangedState()
  {
    if(GoalReached())
      return _fields->at_goal_state;

    return _fields->take_aim_state;
  }


  void MotionPlannerState::
  TurnToward(double timestep, direction_t direction,
	     shared_ptr<const sfl::Scan> global_scan)
    const
  {
    _fields->dynamicWindow.GoSlow();
    AskDynamicWindow(timestep, direction, global_scan);
  }


  void MotionPlannerState::
  GoAlong(double timestep, direction_t direction,
	  shared_ptr<const sfl::Scan> global_scan)
    const
  {
    _fields->dynamicWindow.GoFast();
    AskDynamicWindow(timestep, direction, global_scan);
  }


  MotionPlannerState::direction_t MotionPlannerState::
  AskBubbleBand() const
  {
    _fields->bubbleBand.Update();
    double goalx(_fields->goal.X());
    double goaly(_fields->goal.Y());
    if(_fields->bubbleBand.GetState() != sfl::BubbleBand::NOBAND)
      _fields->bubbleBand.GetSubGoal(_fields->bubbleBand.robot_radius,
				     goalx, goaly);
    _fields->odometry.Get()->From(goalx, goaly);
    return make_pair(goalx, goaly);
  }


  void MotionPlannerState::
  AskDynamicWindow(double timestep,
		   direction_t direction,
		   shared_ptr<const sfl::Scan> global_scan)
    const
  {
    _fields->dynamicWindow.Update(timestep, direction.first, direction.second,
				  global_scan);
    
    double qdl, qdr;
    if( ! _fields->dynamicWindow.OptimalActuators(qdl, qdr)){
      qdl = 0;
      qdr = 0;
    }
  
    _fields->motionController.ProposeActuators(qdl, qdr);
  }


  TakeAimState::
  TakeAimState(MotionPlannerFields * fields):
    MotionPlannerState("take aim", fields)
  {
  }


  void TakeAimState::
  Act(double timestep, shared_ptr<const sfl::Scan> global_scan)
  {
    TurnToward(timestep, GetPathDirection(), global_scan);
  }


  MotionPlannerState * TakeAimState::
  NextState(double timestep)
  {
    MotionPlannerState * override(MotionPlannerState::NextState(timestep));
    if(override != this)
      return override;

    if(StartHoming(dheading))
      return _fields->aimed_state;

    return this;
  }


  MotionPlannerState::direction_t TakeAimState::
  GetPathDirection()
  {
    direction_t dir(AskBubbleBand());
    dheading = atan2(dir.second, dir.first);
    if( ! _fields->goForward)
      if(dheading > 0)
	dheading =   M_PI - dheading;
      else
	dheading = - M_PI - dheading;
    return dir;
  }


  bool TakeAimState::
  StartHoming(double dtheta)
    const
  {
    return (dtheta > 0 ? dtheta : - dtheta) <= DTHETASTARTHOMING;
  }


  AimedState::
  AimedState(MotionPlannerFields * fields):
    MotionPlannerState("aimed", fields)
  {
  }


  void AimedState::
  Act(double timestep, shared_ptr<const sfl::Scan> global_scan)
  {
    GoAlong(timestep, GetPathDirection(), global_scan);
  }


  MotionPlannerState * AimedState::
  NextState(double timestep)
  {
    MotionPlannerState  *override(MotionPlannerState::NextState(timestep));
    if(override != this)
      return override;
  
    if(StartAiming(dheading))
      return _fields->take_aim_state;
  
    return this;
  }


  MotionPlannerState::direction_t AimedState::
  GetPathDirection()
  {
    direction_t dir(AskBubbleBand());
    dheading = atan2(dir.second, dir.first);
    if( ! _fields->goForward)
      if(dheading > 0)
	dheading =   M_PI - dheading;
      else
	dheading = - M_PI - dheading;
    return dir;
  }


  bool AimedState::
  StartAiming(double dtheta)
    const
  {
    return (dtheta > 0 ? dtheta : - dtheta) >= DTHETASTARTAIMING;
  }


  AdjustGoalHeadingState::
  AdjustGoalHeadingState(MotionPlannerFields * fields):
    MotionPlannerState("adjust goal heading", fields)
  {
  }


  void AdjustGoalHeadingState::
  Act(double timestep, shared_ptr<const sfl::Scan> global_scan)
  {
    TurnToward(timestep, GetPathDirection(), global_scan);
  }


  AdjustGoalHeadingState::direction_t AdjustGoalHeadingState::
  GetPathDirection()
  {
    shared_ptr<const sfl::Pose> pose(_fields->odometry.Get());    
    double dtheta;
    _fields->goal.HeadingReached( * pose, _fields->goForward, dtheta);
    return make_pair(cos(dtheta), sin(dtheta));
  }


  AtGoalState::
  AtGoalState(MotionPlannerFields * fields):
    MotionPlannerState("at goal", fields)
  {
  }


  void AtGoalState::
  Act(double timestep, shared_ptr<const sfl::Scan> global_scan)
  {
    TurnToward(timestep, GetPathDirection(), global_scan);
  }


  MotionPlannerState * AtGoalState::
  NextState(double timestep)
  {
    MotionPlannerState  * override(MotionPlannerState::NextState(timestep));
    if(override != this)
      return override;
  
    return this;
  }


  MotionPlannerState::direction_t AtGoalState::
  GetPathDirection()
  {
    return direction_t(1, 0);
  }


  NullState::
  NullState(MotionPlannerFields * fields):
    MotionPlannerState("null", fields)
  {
  }


  void NullState::
  Act(double timestep, shared_ptr<const sfl::Scan> global_scan)
  {
    _fields->motionController.ProposeActuators(0, 0);
  }


  MotionPlannerState * NullState::
  NextState(double timestep)
  {
    return _fields->at_goal_state;
  }


  MotionPlannerState::direction_t NullState::
  GetPathDirection()
  {
    return direction_t(0, 0);
  }

}
