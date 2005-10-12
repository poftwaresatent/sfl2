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


#include "Objective.hpp"
#include "DynamicWindow.hpp"
#include <sfl/util/numeric.hpp>


using namespace std;


namespace sfl {


  Objective::
  Objective(const DynamicWindow & dynamic_window):
    _dynamic_window(dynamic_window),
    _dimension(dynamic_window.Dimension())
  {
    _value = new double*[_dimension];
    for(unsigned int i = 0; i < _dimension; ++i)
      _value[i] = new double[_dimension];
  }



  Objective::
  ~Objective()
  {
    for(unsigned int i = 0; i < _dimension; ++i)
      delete[] _value[i];
    delete[] _value;
  }



  void Objective::
  Rescale(unsigned int qdlMin,
	  unsigned int qdlMax,
	  unsigned int qdrMin,
	  unsigned int qdrMax)
  {
    double min(_maxValue);
    double max(_minValue);
    for(unsigned int l = qdlMin; l <= qdlMax; ++l)
      for(unsigned int r = qdrMin; r <= qdrMax; ++r)
	if(_dynamic_window.Admissible(l, r)){
	  if(_value[l][r] < min)
	    min = _value[l][r];
	  if(_value[l][r] > max)
	    max = _value[l][r];
	}

    if((max - min) < epsilon)
      return;

    double scale(1 / (max - min));
    for(unsigned int l = qdlMin; l <= qdlMax; ++l)
      for(unsigned int r = qdrMin; r <= qdrMax; ++r)
	if(_dynamic_window.Admissible(l, r))
	  _value[l][r] = scale * (_value[l][r] - min);
  }



  double Objective::
  Value(unsigned int qdlIndex,
	unsigned int qdrIndex)
    const
  {
    //    if((qdlIndex < _dimension) && (qdrIndex < _dimension))
    return _value[qdlIndex][qdrIndex];
  }



  double Objective::
  Min(unsigned int qdlMin,
      unsigned int qdlMax,
      unsigned int qdrMin,
      unsigned int qdrMax)
    const
  {
    double min(_maxValue);
    for(unsigned int l = qdlMin; l <= qdlMax; ++l)
      for(unsigned int r = qdrMin; r <= qdrMax; ++r)
	if(_dynamic_window.Admissible(l, r))
	  if(_value[l][r] < min)
	    min = _value[l][r];

    return min;
  }



  double Objective::
  Max(unsigned int qdlMin,
      unsigned int qdlMax,
      unsigned int qdrMin,
      unsigned int qdrMax)
    const
  {
    double max(_minValue);
    for(unsigned int l = qdlMin; l <= qdlMax; ++l)
      for(unsigned int r = qdrMin; r <= qdrMax; ++r)
	if(_dynamic_window.Admissible(l, r))
	  if(_value[l][r] > max)
	    max = _value[l][r];

    return max;
  }

}
