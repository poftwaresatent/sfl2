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
#include <iostream>


using boost::shared_ptr;
using std::pair;
using std::make_pair;


namespace sfl {


  BubbleBand::
  BubbleBand(const RobotModel & robot_model,
	     const Odometry & odometry,
	     BubbleList::Parameters _parameters):
    parameters(_parameters),
    _robot_model(robot_model),
    _odometry(odometry),
    _bubble_factory( * new BubbleFactory()),
    _replan_handler( * new ReplanHandler( * this, odometry, _bubble_factory)),
    _active_blist(new BubbleList(*this, _parameters)),
    _robot_radius(robot_model.GetHull()->CalculateRadius()),
    _robot_diameter(2 * _robot_radius),
    _deletion_diameter(1.8 * _robot_diameter),
    _addition_diameter(1.2 * _robot_diameter),
    _reaction_radius(2.0 * _robot_radius),
    _ignore_radius(0.9 * _robot_radius),
    _ignore_radius2(_ignore_radius * _ignore_radius),
    _replan_request(false),
    _state(NOBAND)
  {
  }


  BubbleBand::
  ~BubbleBand()
  {
    delete _active_blist;
    delete & _replan_handler;
  }
  

  void BubbleBand::
  SetGoal(const Goal & global_goal)
  {
    SetMinIgnoreDistance(0);
    SetNF1GoalRadius(global_goal.Dr());
    _replan_request = true;
    _global_goal = global_goal;
  }


  void BubbleBand::
  SetNF1GoalRadius(double r)
  {
    _nf1_goal_radius = r;
  }


  void BubbleBand::
  SetMinIgnoreDistance(double d)
  {
    _min_ignore_distance = d;
  }


  bool BubbleBand::
  AppendGoal(const Goal & global_goal,
	     shared_ptr<const GlobalScan> scan)
  {
    SetMinIgnoreDistance(0);
    SetNF1GoalRadius(global_goal.Dr());

    _global_goal = global_goal;

    if(_active_blist->m_head == 0){
      _replan_request = true;
      return false;
    }

    Bubble *bubble = _bubble_factory.New(_reaction_radius,
					 _global_goal.X(),
					 _global_goal.Y());

    if(bubble == 0){
      _active_blist->RemoveAll();
      _replan_request = true;
      return false;
    }
  
    Bubble *tail(_active_blist->m_tail);
    bubble->UpdateExternalParameters(scan, _ignore_radius);

    bubble->_ignore_distance = tail->_ignore_distance;

    bubble->_dprevious = Bubble::Distance(*bubble, *tail);
    bubble->_dnext = -1;
  
    if(Bubble::CheckOverlap(*tail, *bubble, _robot_radius)){
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
    
      _active_blist->Append(bubble);
      return true;
    }
  
    _bubble_factory.Delete(bubble);
  
    _active_blist->RemoveAll();
    _replan_request = true;
    return false;
  }


  bool BubbleBand::
  AppendTarget(const Goal & global_goal)
  {
    SetMinIgnoreDistance(global_goal.Dr() + _robot_radius);
    SetNF1GoalRadius(_min_ignore_distance);
  
    _global_goal = global_goal;

    if(_active_blist->m_head == 0){
      _replan_request = true;
      return false;
    }

    Bubble *bubble = _bubble_factory.Clone(_active_blist->m_tail);
    if(bubble != 0){
      bubble->SetMinIgnoreDistance(0);
      bubble->_alpha_int = Bubble::DEFAULTALPHAINT;
      bubble->_alpha_ext = Bubble::DEFAULTALPHAEXT;
      _active_blist->InsertAfter(_active_blist->m_tail->_previous, bubble);
    }

    _active_blist->m_tail->_position = make_pair(_global_goal.X(),
						 _global_goal.Y());

    return true;
  }


  int BubbleBand::
  Update(shared_ptr<const GlobalScan> scan)
  {
    // thread emulation
    _bubble_factory.EmulatedThread();
    _replan_handler.UpdateEmulation(scan);

    // the real mccoy
    UpdateRobotPose();

    if(_replan_request){
      _replan_handler.Abort();
      _replan_request = false;
      _active_blist->RemoveAll();
      _replan_handler.StartNewThread();
      _state = NOBAND;
    }
    else{
      bool newBand(false);

      if(_replan_handler._state == ReplanHandler::EXITSUCCESS){
	_replan_handler._state = ReplanHandler::NOTRUNNING;
	SwapBubbleLists();
	newBand = true;
      }
      else if(_replan_handler._state == ReplanHandler::EXITFAILURE){
	_replan_handler.StartNewThread();
      }

      if(_active_blist->Empty()){
	_state = NOBAND;
      }
      else{
	if(_active_blist->Update(scan)){
	  if(newBand){
	    _state = NEWBAND;
	  }
	  else{
	    _state = VALIDBAND;
	  }
	}
	else{
	  _state = UNSUREBAND;
	  if(_replan_handler._state != ReplanHandler::RUNNING){
	    _replan_handler.StartNewThread();
	  }
	}
      }
    }
  
    return _state;
  }
  

  pair<double, double> BubbleBand::
  GetSubGoal()
    const
  {
    if(_active_blist->m_head == 0)
      return make_pair(_global_goal.X(), _global_goal.Y());

    if(_active_blist->m_head == _active_blist->m_tail)
      return make_pair(_global_goal.X(), _global_goal.Y());

    // paranoid: don't return current pose

    for(Bubble *b(_active_blist->m_head->_next);
	b != 0;
	b = b->_next){
      double dx(b->_position.first - _frame.X());
      double dy(b->_position.second - _frame.Y());
      //    if(dx * dx + dy * dy > 1){	// hax: OH MY GAWD!!!
      if(dx * dx + dy * dy > 0.5){	// hax: OH MY GAWD!!!
	return b->_position;
      }
    }

    return make_pair(_global_goal.X(), _global_goal.Y());
  }


  void BubbleBand::
  UpdateRobotPose()
  {
    _frame.Set(_odometry.Get());
  
    if(_active_blist->m_head != 0){
      _active_blist->m_head->_position.first = _frame.X();
      _active_blist->m_head->_position.second = _frame.Y();
    }
  }


  void BubbleBand::
  SwapBubbleLists()
  {
    BubbleList * tmp(_active_blist);
    _active_blist = _replan_handler._buffer_blist;
    _replan_handler._buffer_blist = tmp;
    tmp->RemoveAll();
  }

}
