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


#include "BubbleBand.hpp"
#include "BubbleFactory.hpp"
#include "ReplanHandler.hpp"
#include <sfl/util/pdebug.hpp>
#include <sfl/api/Scan.hpp>
#include <sfl/api/Odometry.hpp>
#include <sfl/api/RobotModel.hpp>
#include <sfl/api/Multiscanner.hpp>
#include <sfl/api/Pose.hpp>
#include <iostream>


#define PDEBUG PDEBUG_OFF


using namespace boost;
using namespace std;


namespace sfl {
  
  
  BubbleBandThread::
  BubbleBandThread(const string & name)
    : SimpleThread(name)
  {
  }
  
  
  void BubbleBandThread::
  Step()
  {
    if(bubbleBand)
      bubbleBand->DoUpdate(bubbleBand->m_multiscanner.CollectScans());
  }
  
  
  BubbleBand::
  BubbleBand(const RobotModel & robot_model,
	     const Odometry & odometry,
	     const Multiscanner & multiscanner,
	     BubbleList::Parameters _parameters,
	     shared_ptr<RWlock> rwlock)
    : parameters(_parameters),
      robot_radius(robot_model.GetHull()->CalculateRadius()),
      robot_diameter(2 * robot_radius),
      ignore_radius(0.9 * robot_radius),
      deletion_diameter(1.8 * robot_diameter),
      addition_diameter(1.2 * robot_diameter),
      m_odometry(odometry),
      m_multiscanner(multiscanner),
      m_bubble_factory(new BubbleFactory()),
      m_replan_handler(new ReplanHandler(*this, odometry, *m_bubble_factory)),
      m_active_blist(new BubbleList(*this, *m_bubble_factory, _parameters)),
      m_reaction_radius(2.0 * robot_radius),
      m_replan_request(false),
      m_state(NOBAND),
      m_rwlock(rwlock)
  {
  }
  
  
  BubbleBand::
  ~BubbleBand()
  {
    delete m_active_blist;
  }
  

  void BubbleBand::
  SetGoal(const Goal & global_goal)
  {
    m_rwlock->Wrlock();
    m_min_ignore_distance = 0;
    m_nf1_goal_radius = global_goal.Dr();
    m_replan_request = true;
    m_global_goal = global_goal;
    m_rwlock->Unlock();
  }
  
  
  bool BubbleBand::
  AppendGoal(const Goal & global_goal, shared_ptr<const Scan> scan)
  {
    RWlock::wrsentry sentry(m_rwlock);
    
    m_min_ignore_distance = 0;
    m_nf1_goal_radius = global_goal.Dr();
    m_global_goal = global_goal;
    if(m_active_blist->m_head == 0){ // no band to append to...
      m_replan_request = true;
      return false;
    }
    
    // place bubble at new goal, after verifying there's enough space
    Bubble *bubble = m_bubble_factory->New(m_reaction_radius,
					   m_global_goal.X(),
					   m_global_goal.Y());
    if(bubble == 0){
      m_active_blist->RemoveAll();
      m_replan_request = true;
      return false;
    }
    Bubble *tail(m_active_blist->m_tail);
    bubble->UpdateExternalParameters(scan, ignore_radius);
    bubble->_ignore_distance = tail->_ignore_distance;
    bubble->_dprevious = Bubble::Distance(*bubble, *tail);
    bubble->_dnext = -1;
    if(Bubble::CheckOverlap(*tail, *bubble, robot_radius)){
      tail->_alpha_int = Bubble::DEFAULTALPHAINT;
      tail->_alpha_ext = Bubble::DEFAULTALPHAEXT;
      tail->_dnext = bubble->_dprevious;
      tail->_fint.first
	= (bubble->_position.first - tail->_position.first) / tail->_dnext;
      tail->_fint.second
	= (bubble->_position.second - tail->_position.second) / tail->_dnext;
      m_active_blist->Append(bubble);
      return true;
    }
    
    // couldn't fix existing band
    m_bubble_factory->Delete(bubble);
    m_active_blist->RemoveAll();
    m_replan_request = true;
    return false;
  }
  
  
  bool BubbleBand::
  AppendTarget(const Goal & global_goal)
  {
    RWlock::wrsentry sentry(m_rwlock);
    
    m_min_ignore_distance = global_goal.Dr() + robot_radius;
    m_nf1_goal_radius = m_min_ignore_distance;
    m_global_goal = global_goal;
    if(m_active_blist->m_head == 0){
      m_replan_request = true;
      return false;
    }
    
    // cheap method: move the last bubble onto the target, attempt to
    // keep a copy on its previous position, don't even check if
    // there's enough clearance (next update should take care of that...)
    Bubble *bubble = m_bubble_factory->Clone(m_active_blist->m_tail);
    if(bubble != 0){
      bubble->SetMinIgnoreDistance(0);
      bubble->_alpha_int = Bubble::DEFAULTALPHAINT;
      bubble->_alpha_ext = Bubble::DEFAULTALPHAEXT;
      m_active_blist->InsertAfter(m_active_blist->m_tail->_previous, bubble);
    }
    m_active_blist->m_tail->_position =
      make_pair(m_global_goal.X(), m_global_goal.Y());
    return true;
  }
  
  
  void BubbleBand::
  Update()
  {
    if( ! m_thread)
      DoUpdate(m_multiscanner.CollectScans());
  }
  
  
  void BubbleBand::
  DoUpdate(shared_ptr<const Scan> scan)
  {
    RWlock::wrsentry sentry(m_rwlock);
    
    m_bubble_factory->EmulatedThread();
    
    if(m_replan_request){
      PDEBUG("replan request\n");
      m_replan_request = false;
      m_replan_handler->Abort();
      m_active_blist->RemoveAll();
      m_replan_handler->StartPlanning();
      m_state = NOBAND;
      return;
    }
    
    m_frame = * m_odometry.Get();
    if(m_active_blist->m_head != 0){
      m_active_blist->m_head->_position.first = m_frame.X();
      m_active_blist->m_head->_position.second = m_frame.Y();
    }
    
    m_replan_handler->Update(scan);
    const ReplanHandler::state_t rh_state(m_replan_handler->GetState());
    bool newBand(false);
    if(rh_state == ReplanHandler::EXITSUCCESS){
      PDEBUG("replan success: use new band\n");
      m_active_blist = m_replan_handler->SwapBubbleList(m_active_blist);
      newBand = true;
    }
    else if(rh_state == ReplanHandler::EXITFAILURE){
      PDEBUG("replan FAILURE: request new plan, keep old band\n");
      m_replan_handler->StartPlanning();
    }
    else
      PDEBUG("replan state %s: keep old band\n",
	     ReplanHandler::GetStateName(rh_state).c_str());
    
    if(m_active_blist->Empty()){
      PDEBUG("active band is EMPTY\n");
      m_state = NOBAND;
    }
    else{
      if(m_active_blist->Update(scan)){
	PDEBUG("active band updated with success\n");
	if(newBand)
	  m_state = NEWBAND;
	else
	  m_state = VALIDBAND;
      }
      else{
	PDEBUG("active band update FAILED: request replan\n");
	m_state = UNSUREBAND;
	m_replan_handler->StartPlanning();
      }
    }
  }
  
  
  void BubbleBand::
  GetSubGoal(double carrot_distance, double & goalx, double & goaly) const
  {
    RWlock::rdsentry sentry(m_rwlock);
    
    if((m_active_blist->m_head == 0)
       || (m_active_blist->m_head == m_active_blist->m_tail)){
      goalx = m_global_goal.X();
      goaly = m_global_goal.Y();
      return;
    }
    
    carrot_distance *= carrot_distance; // avoid sqrt() calls
    for(Bubble *b(m_active_blist->m_head->_next); b != 0; b = b->_next){
      double dx(b->_position.first - m_frame.X());
      double dy(b->_position.second - m_frame.Y());
      if(dx * dx + dy * dy >= carrot_distance){
	goalx = b->_position.first;
	goaly = b->_position.second;
	return;
      }
    }
    
    goalx = m_global_goal.X();
    goaly = m_global_goal.Y();
  }
  
  
  bool BubbleBand::
  SetThread(shared_ptr<BubbleBandThread> thread)
  {
    RWlock::wrsentry sentry(m_rwlock);
    if(m_thread)
      return false;
    m_thread = thread;
    thread->bubbleBand = this;
    return true;
  }

}
