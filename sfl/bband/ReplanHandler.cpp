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


using std::pair;
using std::make_pair;


namespace sfl {


  ReplanHandler::
  ReplanHandler(BubbleBand & bubble_band,
		const Odometry & odometry,
		BubbleFactory & bubble_factory):
    _bubble_band(bubble_band),
    _odometry(odometry),
    _bubble_factory(bubble_factory),
    _nf1(* new NF1()),
    _buffer_blist(new BubbleList(bubble_band, bubble_band.parameters)),
    _initial_band(0),
    _nf1width(DEFAULTNF1WIDTH),
    _nf1dimension(DEFAULTNF1DIMENSION),
    _state(NOTRUNNING),
    _threadcounter(-1),
    _breakpoint(0)
  {
    _statestr[NOTRUNNING]  = "NOTRUNNING";
    _statestr[RUNNING]     = "RUNNING";
    _statestr[EXITSUCCESS] = "EXITSUCCESS";
    _statestr[EXITFAILURE] = "EXITFAILURE";
    _statestr[ABORTED]     = "ABORTED";
  }


  ReplanHandler::
  ~ReplanHandler()
  {
    if(_buffer_blist)
      delete _buffer_blist;
    delete &_nf1;
    if(_initial_band)
      delete _initial_band;
  }


  void ReplanHandler::
  UpdateEmulation(boost::shared_ptr<const GlobalScan> scan)
  {
    static const int TRIGGER_NF1   = 1;
    static const int TRIGGER_IBAND = 2;
    static const int TRIGGER_UBAND = 3;

    if(_state != RUNNING){
      return;
    }

    if( ! ++_threadcounter) // first iteration directly after StartNewThread()
      _breakpoint = 0;

    if((_threadcounter == TRIGGER_NF1)   ||
       (_threadcounter == TRIGGER_IBAND) ||
       (_threadcounter == TRIGGER_UBAND)){
      _state = Run(_breakpoint, scan);
    }
  }


  void ReplanHandler::
  StartNewThread()
  {
    _state = RUNNING;
    _threadcounter = -1;
  }


  void ReplanHandler::
  Abort()
  {
    if(_state == RUNNING)
      _state = ABORTED;
    else
      _state = NOTRUNNING;
  }


  int ReplanHandler::
  Run(int & breakpoint,
      boost::shared_ptr<const GlobalScan> scan)
  {
    int result = RUNNING;

    switch(breakpoint){

    case 0:
      if( ! GeneratePlan(scan)){
	_nf1width *= 2;
	result = EXITFAILURE;
      }
      else{
	_nf1width = DEFAULTNF1WIDTH;
	breakpoint++;
      }
      break;

    case 1:
      if( ! GenerateBand(scan)){
	result = EXITFAILURE;
      }
      else{
	breakpoint++;

	// keep a copy for plotting
	if(_initial_band)
	  delete _initial_band;
	_initial_band = new BubbleList( * _buffer_blist);
      }
      break;
    
    case 2:
      if( ! _buffer_blist->Update(scan)){
	result = EXITFAILURE;
      }
      else{
	breakpoint++;
	result = EXITSUCCESS;
      }
      break;
    
    default:
      abort();
    }

    return result;
  }


  bool ReplanHandler::
  GeneratePlan(boost::shared_ptr<const GlobalScan> scan)
  {
    _nf1.Configure(make_pair(_odometry.Get().X(), _odometry.Get().Y()),
		   make_pair(_bubble_band.GlobalGoal().X(),
			     _bubble_band.GlobalGoal().Y()),
		   _nf1width,
		   _nf1dimension);
  
    _nf1.Initialize(scan,
		    _bubble_band.RobotRadius(),
		    _bubble_band.NF1GoalRadius());
  
    _nf1.Calculate();
  
    return _nf1.ResetTrace();
  }


  bool ReplanHandler::
  GenerateBand(boost::shared_ptr<const GlobalScan> scan)
  {
    _buffer_blist->RemoveAll();
    
    Bubble *bubble(_bubble_factory.New(_bubble_band.ReactionRadius(),
				       _bubble_band.RobotPose().X(),
				       _bubble_band.RobotPose().Y()));
    if(bubble == 0)
      return false;
    
    bubble->_alpha_int = 0;
    bubble->_alpha_ext = 0;

    _buffer_blist->Append(bubble);
  
    pair<double, double> point;
    while(_nf1.GlobalTrace(point)){
      bubble = _bubble_factory.New(_bubble_band.ReactionRadius(),
				   point.first,
				   point.second);

      if(bubble == 0){
	return false;
      }

      _buffer_blist->Append(bubble);
    }

    bubble = _bubble_factory.New(_bubble_band.ReactionRadius(),
				 _bubble_band.GlobalGoal().X(),
				 _bubble_band.GlobalGoal().Y());
    if(bubble == 0){
      return false;
    }

    bubble->_alpha_int = 0;
    bubble->_alpha_ext = 0;
    bubble->SetMinIgnoreDistance(_bubble_band.MinIgnoreDistance());

    _buffer_blist->Append(bubble);

    if( ! _buffer_blist->Update(scan)){
      return false;
    }

    return true;
  }

}
