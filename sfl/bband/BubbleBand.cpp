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
#include <sfl/api/GlobalScan.hpp>
#include <sfl/api/Odometry.hpp>
#include <sfl/api/RobotModel.hpp>
#include <iostream>


//#define DEBUG_SFL_BUBBLE_BAND
#ifdef DEBUG_SFL_BUBBLE_BAND
using std::cout;
#endif // DEBUG_SFL_BUBBLE_BAND


using boost::shared_ptr;
using std::pair;
using std::make_pair;


namespace sfl {
  
  
  BubbleBand::
  BubbleBand(const RobotModel & robot_model,
	     const Odometry & odometry,
	     BubbleList::Parameters _parameters):
    parameters(_parameters),
    robot_radius(robot_model.GetHull()->CalculateRadius()),
    robot_diameter(2 * robot_radius),
    ignore_radius(0.9 * robot_radius),
    deletion_diameter(1.8 * robot_diameter),
    addition_diameter(1.2 * robot_diameter),
    m_odometry(odometry),
    m_bubble_factory(new BubbleFactory()),
    m_replan_handler(new ReplanHandler(*this, odometry, *m_bubble_factory)),
    m_active_blist(new BubbleList(*this, *m_bubble_factory, _parameters)),
    m_reaction_radius(2.0 * robot_radius),
    //    m_ignore_radius2(ignore_radius * ignore_radius),
    m_replan_request(false),
    m_state(NOBAND)
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
    SetMinIgnoreDistance(0);
    SetNF1GoalRadius(global_goal.Dr());
    m_replan_request = true;
    m_global_goal = global_goal;
  }


  void BubbleBand::
  SetNF1GoalRadius(double r)
  {
    m_nf1_goal_radius = r;
  }


  void BubbleBand::
  SetMinIgnoreDistance(double d)
  {
    m_min_ignore_distance = d;
  }


  bool BubbleBand::
  AppendGoal(const Goal & global_goal,
	     shared_ptr<const GlobalScan> scan)
  {
    SetMinIgnoreDistance(0);
    SetNF1GoalRadius(global_goal.Dr());

    m_global_goal = global_goal;

    if(m_active_blist->m_head == 0){
      m_replan_request = true;
      return false;
    }

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

      // hax: should check for distance < epsilon...
      tail->_dnext = bubble->_dprevious;

      tail->_fint.first
	= (bubble->_position.first - tail->_position.first)
	/ tail->_dnext;

      tail->_fint.second
	= (bubble->_position.second - tail->_position.second)
	/ tail->_dnext;
    
      m_active_blist->Append(bubble);
      return true;
    }
  
    m_bubble_factory->Delete(bubble);
  
    m_active_blist->RemoveAll();
    m_replan_request = true;
    return false;
  }


  bool BubbleBand::
  AppendTarget(const Goal & global_goal)
  {
    SetMinIgnoreDistance(global_goal.Dr() + robot_radius);
    SetNF1GoalRadius(m_min_ignore_distance);
  
    m_global_goal = global_goal;

    if(m_active_blist->m_head == 0){
      m_replan_request = true;
      return false;
    }

    Bubble *bubble = m_bubble_factory->Clone(m_active_blist->m_tail);
    if(bubble != 0){
      bubble->SetMinIgnoreDistance(0);
      bubble->_alpha_int = Bubble::DEFAULTALPHAINT;
      bubble->_alpha_ext = Bubble::DEFAULTALPHAEXT;
      m_active_blist->InsertAfter(m_active_blist->m_tail->_previous, bubble);
    }

    m_active_blist->m_tail->_position = make_pair(m_global_goal.X(),
						  m_global_goal.Y());

    return true;
  }


  void BubbleBand::
  Update(shared_ptr<const GlobalScan> scan)
  {
    m_bubble_factory->EmulatedThread();
    
#ifdef DEBUG_SFL_BUBBLE_BAND
    cout << "DEBUG BubbleBand::Update():\n";
#endif // DEBUG_SFL_BUBBLE_BAND
    
    if(m_replan_request){
#ifdef DEBUG_SFL_BUBBLE_BAND
      cout << "  replan request!\n";
#endif // DEBUG_SFL_BUBBLE_BAND
      m_replan_request = false;
      m_replan_handler->Abort();
      m_active_blist->RemoveAll();
      m_replan_handler->StartPlanning();
      m_state = NOBAND;
      return;
    }
    
    m_replan_handler->Update(scan);
    // Beware of latch-"feature" inside GetState(), which can change
    // the state of the replan handler. BAD DESIGN!
    const ReplanHandler::state_t rh_state(m_replan_handler->GetState());
    UpdateRobotPose();
    
    bool newBand(false);
    if(rh_state == ReplanHandler::EXITSUCCESS){
#ifdef DEBUG_SFL_BUBBLE_BAND
      cout << "  replan success: use new band\n";
#endif // DEBUG_SFL_BUBBLE_BAND
      m_active_blist = m_replan_handler->SwapBubbleList(m_active_blist);
      newBand = true;
    }
    else if(rh_state == ReplanHandler::EXITFAILURE){
#ifdef DEBUG_SFL_BUBBLE_BAND
      cout << "  replan FAILURE: request new plan, keep old band\n";
#endif // DEBUG_SFL_BUBBLE_BAND
      m_replan_handler->StartPlanning();
    }
#ifdef DEBUG_SFL_BUBBLE_BAND
    else
      cout << "  replan state is \""
	   << ReplanHandler::GetStateName(rh_state)
	   << "\": keep old band\n";
#endif // DEBUG_SFL_BUBBLE_BAND
    
    if(m_active_blist->Empty()){
#ifdef DEBUG_SFL_BUBBLE_BAND
      cout << "  active band is EMPTY\n";
#endif // DEBUG_SFL_BUBBLE_BAND
      m_state = NOBAND;
    }
    else{
      if(m_active_blist->Update(scan)){
#ifdef DEBUG_SFL_BUBBLE_BAND
	cout << "  active band updated with success\n";
#endif // DEBUG_SFL_BUBBLE_BAND
	if(newBand)
	  m_state = NEWBAND;
	else
	  m_state = VALIDBAND;
      }
      else{
#ifdef DEBUG_SFL_BUBBLE_BAND
	cout << "  active band update FAILED: request replan\n";
#endif // DEBUG_SFL_BUBBLE_BAND
	m_state = UNSUREBAND;
	m_replan_handler->StartPlanning();
      }
    }
  }
  

  pair<double, double> BubbleBand::
  GetSubGoal()
    const
  {
    if(m_active_blist->m_head == 0)
      return make_pair(m_global_goal.X(), m_global_goal.Y());

    if(m_active_blist->m_head == m_active_blist->m_tail)
      return make_pair(m_global_goal.X(), m_global_goal.Y());

    // paranoid: don't return current pose

    for(Bubble *b(m_active_blist->m_head->_next);
	b != 0;
	b = b->_next){
      double dx(b->_position.first - m_frame.X());
      double dy(b->_position.second - m_frame.Y());
      //    if(dx * dx + dy * dy > 1){	// hax: OH MY GAWD!!!
      if(dx * dx + dy * dy > 0.5){	// hax: OH MY GAWD!!!
	return b->_position;
      }
    }

    return make_pair(m_global_goal.X(), m_global_goal.Y());
  }


  void BubbleBand::
  UpdateRobotPose()
  {
    m_frame = * m_odometry.Get();
      if(m_active_blist->m_head != 0){
      m_active_blist->m_head->_position.first = m_frame.X();
      m_active_blist->m_head->_position.second = m_frame.Y();
    }
  }
  
}
