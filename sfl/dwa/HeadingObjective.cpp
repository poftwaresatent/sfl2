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


#include "HeadingObjective.hpp"
#include "DynamicWindow.hpp"
#include <cmath>
using namespace std;


namespace sfl {


  HeadingObjective::
  HeadingObjective(const DynamicWindow & dynamic_window,
		   const RobotModel & robot_model):
    Objective(dynamic_window),
    _robot_model(robot_model),
    _offset(0)
  {
    _standstill_prediction = new Frame*[_dimension];
    for(unsigned int i = 0; i < _dimension; ++i)
      _standstill_prediction[i] = new Frame[_dimension];

    _step_prediction = new Frame*[_dimension];
    for(unsigned int i = 0; i < _dimension; ++i)
      _step_prediction[i] = new Frame[_dimension];
  }


  HeadingObjective::
  ~HeadingObjective()
  {
    for(unsigned int i = 0; i < _dimension; ++i)
      delete[] _standstill_prediction[i];
    delete[] _standstill_prediction;

    for(unsigned int i = 0; i < _dimension; ++i)
      delete[] _step_prediction[i];
    delete[] _step_prediction;
  }


  void HeadingObjective::
  Initialize(ostream * progress_stream)
  {
    for(unsigned int iqdl = 0; iqdl < _dimension; ++iqdl){
      for(unsigned int iqdr = 0; iqdr < _dimension; ++iqdr){
	double qdl(_dynamic_window.Qd(iqdl));
	double qdr(_dynamic_window.Qd(iqdr));

	double sd, thetad;
	_robot_model.Actuator2Global(qdl, qdr, sd, thetad);

	double stoptime;
	if(absval(qdl) > absval(qdr))
	  stoptime = absval(qdl) / _robot_model.QddMax();
	else
	  stoptime = absval(qdr) / _robot_model.QddMax();
	stoptime += _robot_model.Timestep();

	double x, y, theta;
	_robot_model.LocalKinematics(sd, thetad,
				     stoptime,
				     x, y, theta);
	_standstill_prediction[iqdl][iqdr].Set(x, y, theta);

	_robot_model.LocalKinematics(sd, thetad,
				     _robot_model.Timestep(), 
				     x, y, theta);
	_step_prediction[iqdl][iqdr].Set(x, y, theta);
      }
    }
  }


  void HeadingObjective::
  Calculate(unsigned int qdlMin,
	    unsigned int qdlMax,
	    unsigned int qdrMin,
	    unsigned int qdrMax)
  {
    for(unsigned int l = qdlMin; l <= qdlMax; ++l)
      for(unsigned int r = qdrMin; r <= qdrMax; ++r)
	if(_dynamic_window.Admissible(l, r)){
	  double x(_dx);
	  double y(_dy);
	  _standstill_prediction[l][r].From(x, y);
	  double dtheta(mod2pi(atan2(y, x) + _offset));
	  _value[l][r] = 1 - absval(dtheta) / M_PI;
	}
	else
	  _value[l][r] = _minValue;
  }


  void HeadingObjective::
  SetGoal(double lx,
	  double ly)
  {
    _dx = lx;
    _dy = ly;
  }


  void HeadingObjective::
  GetGoal(double & lx,
	  double & ly)
    const
  {
    lx = _dx;
    ly = _dy;
  }


  void HeadingObjective::
  SetOffset(double angle)
  {
    _offset = angle;
  }


  double HeadingObjective::
  GetOffset()
    const
  {
    return _offset;
  }


  const Frame & HeadingObjective::
  PredictedStandstill(unsigned int iqdl,
		      unsigned int iqdr)
    const
  {
    return _standstill_prediction[iqdl][iqdr];
  }


  const Frame & HeadingObjective::
  PredictedStep(unsigned int iqdl,
		unsigned int iqdr)
    const
  {
    return _step_prediction[iqdl][iqdr];
  }

}
