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


#include "SpeedObjective.hpp"
#include "DynamicWindow.hpp"
using namespace std;


namespace sfl {


SpeedObjective::
SpeedObjective(const DynamicWindow & dynamic_window,
	       const RobotModel & robot_model):
  Objective(dynamic_window),
  _sdMax(robot_model.SdMax()),
  _robot_model(robot_model),
  _goForward(true),
  _forward(_dimension, _minValue, _maxValue),
  _backward(_dimension, _minValue, _maxValue),
  _slow(_dimension, _minValue, _maxValue)
{
  _sd = new double*[_dimension];
  for(unsigned int i = 0; i < _dimension; ++i)
    _sd[i] = new double[_dimension];

  _thetad = new double*[_dimension];
  for(unsigned int i = 0; i < _dimension; ++i)
    _thetad[i] = new double[_dimension];

  _current = & _forward;
}



SpeedObjective::
~SpeedObjective()
{
  for(unsigned int i = 0; i < _dimension; ++i)
    delete[] _sd[i];
  delete[] _sd;

  for(unsigned int i = 0; i < _dimension; ++i)
    delete[] _thetad[i];
  delete[] _thetad;
}



void SpeedObjective::
Initialize(ostream * progress_stream)
{
  for(unsigned int l = 0; l < _dimension; ++l)
    for(unsigned int r = 0; r < _dimension; ++r)
      _robot_model.Actuator2Global(_dynamic_window.Qd(l),
				   _dynamic_window.Qd(r),
				  _sd[l][r],
				   _thetad[l][r]);

  for(unsigned int r = 0; r < _dimension; ++r){
    if((r % 2) != 0)
      _forward.LoadBuffer(0,
			  r,
			  _minValue +
			  (_maxValue - _minValue) *
			  (_sd[0][r] + _sdMax) /
			  (2 * _sdMax));
    
    for(unsigned int l = r % 2; l < _dimension; l += 2){
      double val(_minValue +
		 (_maxValue - _minValue) *
		 (_sd[l][r] + _sdMax) /
		 (2 * _sdMax));
      _forward.LoadBuffer(l, r, val);
      if(l < _dimension - 1)
	_forward.LoadBuffer(l + 1, r, val);	
    }
  }
  _forward.SaveBuffer();

  for(unsigned int r = 0; r < _dimension; ++r){
    if((r % 2) != 0)
      _backward.LoadBuffer(_dimension - 1,
			   r,
			   _maxValue +
			   _minValue -
			   (_maxValue - _minValue) *
			   (_sd[_dimension - 1][r] + _sdMax) /
			   (2 * _sdMax));
    for(unsigned int l = r % 2; l < _dimension; l += 2){
      double val(_maxValue +
		 _minValue -
		 (_maxValue - _minValue) *
		 (_sd[l][r] + _sdMax) /
		 (2 * _sdMax));
      _backward.LoadBuffer(l, r, val);
      if(l > 0)
	_backward.LoadBuffer(l - 1, r, val);	
    }
  }
  _backward.SaveBuffer();
  
  for(unsigned int l = 0; l < _dimension; ++l)
    for(unsigned int r = 0; r < _dimension; ++r)
      _slow.LoadBuffer(l,
		       r,
		       _maxValue -
		       (_maxValue - _minValue) *
		       absval(_sd[l][r]) /
		       _sdMax);
  _slow.SaveBuffer();
}



void SpeedObjective::
Calculate(unsigned int qdlMin,
	  unsigned int qdlMax,
	  unsigned int qdrMin,
	  unsigned int qdrMax)
{
  for(unsigned int l = qdlMin; l <= qdlMax; ++l)
    for(unsigned int r = qdrMin; r <= qdrMax; ++r)
      if(_dynamic_window.Admissible(l, r))
	_value[l][r] = _current->Get(l, r);
      else
	_value[l][r] = _minValue;
}



void SpeedObjective::
GoFast()
{
  if(_goForward)
    _current = & _forward;
  else
    _current = & _backward;
}



void SpeedObjective::
GoSlow()
{
  _current = & _slow;
}



void SpeedObjective::
GoForward()
{
  _goForward = true;
}



void SpeedObjective::
GoBackward()
{
  _goForward = false;
}



}
