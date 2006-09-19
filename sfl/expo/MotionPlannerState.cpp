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
#include "MotionPlanner.hpp"
#include "MotionController.hpp"
#include <sfl/util/pdebug.hpp>
#include <sfl/api/Pose.hpp>
#include <sfl/api/Odometry.hpp>
#include <sfl/bband/BubbleBand.hpp>
#include <sfl/dwa/DynamicWindow.hpp>
#include <cmath>


#ifdef DEBUG
# define PDEBUG PDEBUG_ERR
# define PVDEBUG PDEBUG_OFF
#else // ! DEBUG
# define PDEBUG PDEBUG_OFF
# define PVDEBUG PDEBUG_OFF
#endif // DEBUG


using namespace boost;
using namespace std;


namespace expo {


  const double TakeAimState::DTHETASTARTHOMING(10 * M_PI / 180);
  const double AimedState::DTHETASTARTAIMING(45 * M_PI / 180);


  MotionPlannerState::
  MotionPlannerState(const string & name, MotionPlanner * mp)
    : m_mp(mp),
      m_name(name)
  {
  }


  MotionPlannerState::
  ~MotionPlannerState()
  {
  }


  const string & MotionPlannerState::
  Name() const
  {
    return m_name;
  }


  bool MotionPlannerState::
  GoalReached() const
  {
    shared_ptr<const sfl::Pose> pose(m_mp->odometry->Get());
    return ( ! m_mp->motion_controller->Moving())
      && m_mp->goal->Reached( * pose, true);
  }


  void MotionPlannerState::
  GoForward(bool b)
  {
    m_mp->go_forward = b;
  }


  MotionPlannerState * MotionPlannerState::
  NextState(double timestep)
  {
    shared_ptr<const sfl::Pose> pose(m_mp->odometry->Get());
    if(m_mp->goal->DistanceReached( * pose)){
      double dheading;
      if(m_mp->goal->HeadingReached( * pose, m_mp->go_forward, dheading)
	 && m_mp->motion_controller->Stoppable(timestep)){
	return m_mp->at_goal_state.get();
      }
    
      return m_mp->adjust_goal_heading_state.get();      
    }
 
    return this;
  }


  MotionPlannerState * MotionPlannerState::
  FollowTargetState()
  {
    return m_mp->aimed_state.get();
  }


  MotionPlannerState * MotionPlannerState::
  GoalChangedState()
  {
    if(GoalReached())
      return m_mp->at_goal_state.get();

    return m_mp->take_aim_state.get();
  }


  void MotionPlannerState::
  TurnToward(double timestep, direction_t direction,
	     shared_ptr<const sfl::Scan> global_scan) const
  {
    m_mp->dynamic_window->GoSlow();
    AskDynamicWindow(timestep, direction, global_scan);
  }


  void MotionPlannerState::
  GoAlong(double timestep, direction_t direction,
	  shared_ptr<const sfl::Scan> global_scan) const
  {
    m_mp->dynamic_window->GoFast();
    AskDynamicWindow(timestep, direction, global_scan);
  }


  MotionPlannerState::direction_t MotionPlannerState::
  AskBubbleBand() const
  {
    double goalx(m_mp->goal->X());
    double goaly(m_mp->goal->Y());
    if(m_mp->bubble_band){
      m_mp->bubble_band->Update();
      if(m_mp->bubble_band->GetState() != sfl::BubbleBand::NOBAND){
	m_mp->bubble_band->GetSubGoal(m_mp->bubble_band->robot_radius,
				      goalx, goaly);
	PDEBUG("expo MPS %s: goal %05.2f %05.2f   bband %05.2f %05.2f\n",
	       m_name.c_str(), m_mp->goal->X(), m_mp->goal->Y(),
	       goalx, goaly);
      }
      else
	PDEBUG("expo MPS %s: goal %05.2f %05.2f  NO BBAND\n",
	       m_name.c_str(), goalx, goaly);
    }
    else
      PDEBUG("expo MPS %s: goal %05.2f %05.2f  bband disabled\n",
	     m_name.c_str(), goalx, goaly);
    m_mp->odometry->Get()->From(goalx, goaly);
    return make_pair(goalx, goaly);
  }


  void MotionPlannerState::
  AskDynamicWindow(double timestep,
		   direction_t direction,
		   shared_ptr<const sfl::Scan> global_scan) const
  {
    m_mp->dynamic_window->Update(timestep, direction.first, direction.second,
				  global_scan);
    
    double qdl, qdr;
    if( ! m_mp->dynamic_window->OptimalActuators(qdl, qdr)){
      qdl = 0;
      qdr = 0;
      PDEBUG("expo MPS %s: DWA failed\n", m_name.c_str());
    }
    else
      PDEBUG("expo MPS %s: DWA %05.2f %05.2f\n", m_name.c_str(), qdl, qdr);
    
    m_mp->motion_controller->ProposeActuators(qdl, qdr);
  }


  TakeAimState::
  TakeAimState(MotionPlanner * mp):
    MotionPlannerState("take aim", mp)
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
      return m_mp->aimed_state.get();

    return this;
  }


  MotionPlannerState::direction_t TakeAimState::
  GetPathDirection()
  {
    direction_t dir(AskBubbleBand());
    dheading = atan2(dir.second, dir.first);
    if( ! m_mp->go_forward)
      if(dheading > 0)
	dheading =   M_PI - dheading;
      else
	dheading = - M_PI - dheading;
    return dir;
  }


  bool TakeAimState::
  StartHoming(double dtheta) const
  {
    return (dtheta > 0 ? dtheta : - dtheta) <= DTHETASTARTHOMING;
  }


  AimedState::
  AimedState(MotionPlanner * mp):
    MotionPlannerState("aimed", mp)
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
      return m_mp->take_aim_state.get();
  
    return this;
  }


  MotionPlannerState::direction_t AimedState::
  GetPathDirection()
  {
    direction_t dir(AskBubbleBand());
    dheading = atan2(dir.second, dir.first);
    if( ! m_mp->go_forward)
      if(dheading > 0)
	dheading =   M_PI - dheading;
      else
	dheading = - M_PI - dheading;
    return dir;
  }


  bool AimedState::
  StartAiming(double dtheta) const
  {
    return (dtheta > 0 ? dtheta : - dtheta) >= DTHETASTARTAIMING;
  }


  AdjustGoalHeadingState::
  AdjustGoalHeadingState(MotionPlanner * mp):
    MotionPlannerState("adjust goal heading", mp)
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
    shared_ptr<const sfl::Pose> pose(m_mp->odometry->Get());    
    double dtheta;
    m_mp->goal->HeadingReached( * pose, m_mp->go_forward, dtheta);
    return make_pair(cos(dtheta), sin(dtheta));
  }


  AtGoalState::
  AtGoalState(MotionPlanner * mp):
    MotionPlannerState("at goal", mp)
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
  NullState(MotionPlanner * mp):
    MotionPlannerState("null", mp)
  {
  }


  void NullState::
  Act(double timestep, shared_ptr<const sfl::Scan> global_scan)
  {
    m_mp->motion_controller->ProposeActuators(0, 0);
  }


  MotionPlannerState * NullState::
  NextState(double timestep)
  {
    return m_mp->at_goal_state.get();
  }


  MotionPlannerState::direction_t NullState::
  GetPathDirection()
  {
    return direction_t(0, 0);
  }

}
