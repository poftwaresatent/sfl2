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


// manual override
#define DEBUG

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
    if( ! m_mp->goal->Reached( * pose, m_mp->go_forward))
      return false;
    if(m_mp->goal->IsVia())
      return true;
    return ! m_mp->motion_controller->Moving();
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
	 && (m_mp->motion_controller->Stoppable(timestep)
	     || m_mp->goal->IsVia()))
	return m_mp->at_goal_state.get();
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
  GoalChangedState(double timestep)
  {
    shared_ptr<const sfl::Pose> pose(m_mp->odometry->Get());
    if(m_mp->goal->DistanceReached( * pose)){
      double dheading;
      if(m_mp->goal->HeadingReached( * pose, m_mp->go_forward, dheading)
	 && (m_mp->motion_controller->Stoppable(timestep)
	     || m_mp->goal->IsVia()))
	return m_mp->at_goal_state.get();
      return m_mp->adjust_goal_heading_state.get();      
    }
    const double dx(m_mp->goal->X() - pose->X());
    const double dy(m_mp->goal->Y() - pose->Y());
    double dtheta(sfl::mod2pi(atan2(dy, dx) - pose->Theta()));
    if( ! m_mp->go_forward)
      dtheta = M_PI - dtheta;
    if(dtheta < m_mp->dtheta_startaiming)
      return m_mp->aimed_state.get();
    return m_mp->take_aim_state.get();
  }


  void MotionPlannerState::
  TurnToward(double timestep, direction_t direction,
	     shared_ptr<const sfl::Scan> global_scan) const
  {
// //     if(m_mp->strict_dwa)
// //       m_mp->dynamic_window->GoStrictSlow();
// //     else
      m_mp->dynamic_window->GoSlow();
    AskDynamicWindow(timestep, direction, global_scan);
  }


  void MotionPlannerState::
  GoAlong(double timestep, direction_t direction,
	  shared_ptr<const sfl::Scan> global_scan) const
  {
// //     if(m_mp->strict_dwa)
// //       m_mp->dynamic_window->GoStrictFast();
// //     else
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
    sfl::DynamicWindow & dwa(*m_mp->dynamic_window);
    dwa.Update(timestep, direction.first, direction.second, global_scan);
    if(m_mp->auto_adapt_dwa){
      PDEBUG("**************************************************\n");
      const sfl::DistanceObjective & dobj(dwa.GetDistanceObjective());
      static int countdown(0);	// BAD DOG, NO BISCUIT!!!
      if(dobj.GetNNear() > 0){
	PDEBUG("expo MPS %s: near points, auto adapt DWA\n", m_name.c_str());
	dwa.alpha_distance = 1;
	dwa.alpha_speed = 0;
	dwa.alpha_heading = 0;
	countdown = 10;
      }
      else{
	if(countdown > 0){
	  --countdown;
	  PDEBUG("expo MPS %s: auto adapt countdown %d\n",
		 m_name.c_str(), countdown);
	}
	else{
	  dwa.alpha_distance = m_mp->orig_alpha_distance;
	  dwa.alpha_speed = m_mp->orig_alpha_speed;
	  dwa.alpha_heading = m_mp->orig_alpha_heading;
	  PDEBUG("expo MPS %s: original DWA %05.2f %05.2f %05.2f\n",
		 m_name.c_str(), dwa.alpha_distance, dwa.alpha_speed,
		 dwa.alpha_heading);
	}
      }
    }
    
    double qdl, qdr;
    if( ! dwa.OptimalActuators(qdl, qdr)){
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
    return (dtheta > 0 ? dtheta : - dtheta) <= m_mp->dtheta_starthoming;
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
    return (dtheta > 0 ? dtheta : - dtheta) >= m_mp->dtheta_startaiming;
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
