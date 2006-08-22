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


#include "ReplanHandler.hpp"
#include "BubbleList.hpp"
#include "BubbleBand.hpp"
#include "BubbleFactory.hpp"
#include <sfl/api/Pose.hpp>
#include <sfl/gplan/NF1Wave.hpp>


using namespace boost;
using namespace std;


namespace sfl {


  ReplanHandler::
  ReplanHandler(BubbleBand & bubble_band,
		const Odometry & odometry,
		BubbleFactory & bubble_factory):
    m_bubble_band(bubble_band),
    m_odometry(odometry),
    m_bubble_factory(bubble_factory),
    m_nf1(new NF1()),
    m_buffer_blist(new BubbleList(bubble_band,
				  bubble_factory,
				  bubble_band.parameters)),
    m_nf1width(DEFAULTNF1WIDTH),
    m_nf1dimension(DEFAULTNF1DIMENSION),
    m_state(NOTRUNNING)
  {
  }


  ReplanHandler::
  ~ReplanHandler()
  {
    delete m_buffer_blist;
  }


  void ReplanHandler::
  Update(boost::shared_ptr<const GlobalScan> scan)
  {
    if(m_state != RUNNING)
      return;
    
    if( ! GeneratePlan(scan)){
      m_nf1width *= 2;
      m_state = EXITFAILURE;
      return;		  // next Update() will attempt with wider NF1
    }
    m_nf1width = DEFAULTNF1WIDTH; // next planning starts with default
    
    if( ! GenerateBand(scan)){
      m_state = EXITFAILURE;
      return;			// try again next Update() with new NF1
    }
    
    // keep a copy for plotting
    m_initial_band.reset(new BubbleList( * m_buffer_blist));
    
    if( ! m_buffer_blist->Update(scan)){
      m_state = EXITFAILURE;
      return;			// try again next Update() with new NF1
    }
    
    m_state = EXITSUCCESS;
  }
  
  
  void ReplanHandler::
  StartPlanning()
  {
    m_state = RUNNING;
  }
  
  
  void ReplanHandler::
  Abort()
  {
    if(m_state == RUNNING)
      m_state = ABORTED;
    else
      m_state = NOTRUNNING;
  }
  
  
  bool ReplanHandler::
  GeneratePlan(boost::shared_ptr<const GlobalScan> scan)
  {
    shared_ptr<const Pose> pose(m_odometry.Get());
    m_nf1->Configure(make_pair(pose->X(), pose->Y()),
		     make_pair(m_bubble_band.GlobalGoal().X(),
			       m_bubble_band.GlobalGoal().Y()),
		     m_nf1width,
		     m_nf1dimension);
    m_nf1->Initialize(scan,
		      m_bubble_band.robot_radius,
		      m_bubble_band.NF1GoalRadius());
    m_nf1->Calculate();
    return m_nf1->ResetTrace();
  }
  
  
  bool ReplanHandler::
  GenerateBand(boost::shared_ptr<const GlobalScan> scan)
  {
    m_buffer_blist->RemoveAll();
    
    Bubble *bubble(m_bubble_factory.New(m_bubble_band.ReactionRadius(),
					m_bubble_band.RobotPose().X(),
					m_bubble_band.RobotPose().Y()));
    if(bubble == 0)
      return false;
    
    bubble->_alpha_int = 0;
    bubble->_alpha_ext = 0;
    
    m_buffer_blist->Append(bubble);
    
    pair<double, double> point;
    while(m_nf1->GlobalTrace(point)){
      bubble = m_bubble_factory.New(m_bubble_band.ReactionRadius(),
				    point.first,
				    point.second);
      
      if(bubble == 0){
	return false;
      }
      
      m_buffer_blist->Append(bubble);
    }
    
    bubble = m_bubble_factory.New(m_bubble_band.ReactionRadius(),
				  m_bubble_band.GlobalGoal().X(),
				  m_bubble_band.GlobalGoal().Y());
    if(bubble == 0){
      return false;
    }
    
    bubble->_alpha_int = 0;
    bubble->_alpha_ext = 0;
    bubble->SetMinIgnoreDistance(m_bubble_band.MinIgnoreDistance());
    
    m_buffer_blist->Append(bubble);
    
    if( ! m_buffer_blist->Update(scan)){
      return false;
    }
    
    return true;
  }
  
  
  const string & ReplanHandler::
  GetStateName(state_t state)
  {
    static string statestr[] = {
      "NOTRUNNING",
      "RUNNING",
      "EXITSUCCESS",
      "EXITFAILURE",
      "ABORTED",
      "<invalid>"
    };
    if((0 <= state) && (ABORTED >= state))
      return statestr[state];
    return statestr[ABORTED + 1];
  }
  

  BubbleList * ReplanHandler::
  SwapBubbleList(BubbleList * replace)
  {
    BubbleList * result(m_buffer_blist);
    replace->RemoveAll();
    m_buffer_blist = replace;
    return result;
  }
  

  ReplanHandler::state_t ReplanHandler::
  GetState()
  {
    if(EXITSUCCESS == m_state){
      m_state = NOTRUNNING;
      return EXITSUCCESS;
    }
    if(EXITFAILURE == m_state){
      m_state = NOTRUNNING;
      return EXITFAILURE;
    }
    return m_state;
  }
  
}
