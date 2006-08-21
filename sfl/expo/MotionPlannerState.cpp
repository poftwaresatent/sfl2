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
#include <sfl/bband/BubbleBand.hpp>


using sfl::GlobalScan;
using boost::shared_ptr;
using namespace std;


namespace expo {


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
    return
      ( ! _fields->motionController.Moving()) &&
      _fields->goal.Reached(_fields->odometry.Get(), true);
  }


  void MotionPlannerState::
  GoForward(bool b)
  {
    _fields->goForward = b;
  }


  MotionPlannerState * MotionPlannerState::
  NextState(double timestep)
  {
    if(_fields->goal.DistanceReached(_fields->odometry.Get())){
      double dheading;
      if(_fields->goal.HeadingReached(_fields->odometry.Get(),
				      _fields->goForward,
				      dheading) &&
	 _fields->motionController.Stoppable(timestep)){
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
	     shared_ptr<const GlobalScan> global_scan)
    const
  {
//     cerr << "HELLO from MotionPlannerState::TurnToward()\n";
    _fields->dynamicWindow.GoSlow();
    AskDynamicWindow(timestep, direction, global_scan);
  }


  void MotionPlannerState::
  GoAlong(double timestep, direction_t direction,
	  shared_ptr<const GlobalScan> global_scan)
    const
  {
//     cerr << "HELLO from MotionPlannerState::GoAlong()\n";
    _fields->dynamicWindow.GoFast();
    AskDynamicWindow(timestep, direction, global_scan);
  }


  MotionPlannerState::direction_t MotionPlannerState::
  AskBubbleBand(shared_ptr<const GlobalScan> global_scan)
    const
  {
    direction_t ret;
    
    _fields->bubbleBand.Update(global_scan);
    if(_fields->bubbleBand.GetState() == sfl::BubbleBand::NOBAND)
      ret = make_pair(_fields->goal.X(), _fields->goal.Y());
    else
      ret = _fields->bubbleBand.GetSubGoal();
    _fields->odometry.Get().From(ret.first, ret.second);

    return ret;
  }


  void MotionPlannerState::
  AskDynamicWindow(double timestep,
		   direction_t direction,
		   shared_ptr<const GlobalScan> global_scan)
    const
  {
//     cerr << "INFO from MotionPlannerState::AskDynamicWindow()\n"
// 	 << "  direction  : (" << direction.first
// 	 << ", " << direction.second << ")\n"
// 	 << "  nscans     : " << global_scan.Nscans() << "\n"
// 	 << "  scan tlower: " << global_scan.GetTlower() << "\n"
// 	 << "  scan tupper: " << global_scan.GetTupper() << "\n";
    
    _fields->dynamicWindow.Update(timestep, direction.first, direction.second,
				  global_scan);
    
    double qdl, qdr;
    if( ! _fields->dynamicWindow.OptimalActuators(qdl, qdr)){
//       cerr << "  ATTENTION   : No optimal actuator command found!\n";
      qdl = 0;
      qdr = 0;
    }
//     cerr << "  speed command : (" << qdl << ", " << qdr << ")\n";
  
    _fields->motionController.ProposeActuators(qdl, qdr);
  }


  TakeAimState::
  TakeAimState(MotionPlannerFields * fields):
    MotionPlannerState("take aim", fields)
  {
  }


  void TakeAimState::
  Act(double timestep, shared_ptr<const GlobalScan> global_scan)
  {
    TurnToward(timestep, GetPathDirection(global_scan), global_scan);
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
  GetPathDirection(shared_ptr<const GlobalScan> global_scan)
  {
    direction_t dir = AskBubbleBand(global_scan);
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
  Act(double timestep, shared_ptr<const GlobalScan> global_scan)
  {
    GoAlong(timestep, GetPathDirection(global_scan), global_scan);
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
  GetPathDirection(shared_ptr<const GlobalScan> global_scan)
  {
    direction_t dir(AskBubbleBand(global_scan));
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
  Act(double timestep, shared_ptr<const GlobalScan> global_scan)
  {
    TurnToward(timestep, GetPathDirection(global_scan), global_scan);
  }


  AdjustGoalHeadingState::direction_t AdjustGoalHeadingState::
  GetPathDirection(shared_ptr<const GlobalScan> global_scan)
  {
    double dtheta;
    _fields->goal.HeadingReached(_fields->odometry.Get(),
				 _fields->goForward,
				 dtheta);
    return make_pair(cos(dtheta), sin(dtheta));
  }


  AtGoalState::
  AtGoalState(MotionPlannerFields * fields):
    MotionPlannerState("at goal", fields)
  {
  }


  void AtGoalState::
  Act(double timestep, shared_ptr<const GlobalScan> global_scan)
  {
    TurnToward(timestep, GetPathDirection(global_scan), global_scan);
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
  GetPathDirection(shared_ptr<const GlobalScan> global_scan)
  {
    return direction_t(1, 0);
  }


  NullState::
  NullState(MotionPlannerFields * fields):
    MotionPlannerState("null", fields)
  {
  }


  void NullState::
  Act(double timestep, shared_ptr<const GlobalScan> global_scan)
  {
    _fields->motionController.ProposeActuators(0, 0);
  }


  MotionPlannerState * NullState::
  NextState(double timestep)
  {
    return _fields->at_goal_state;
  }


  MotionPlannerState::direction_t NullState::
  GetPathDirection(shared_ptr<const GlobalScan> global_scan)
  {
    return direction_t(0, 0);
  }

}
