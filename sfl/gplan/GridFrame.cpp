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


#include "GridFrame.hpp"
#include <cmath>
using namespace std;



namespace sfl {



  GridFrame::
  GridFrame()
  {
    Configure(0, 0, 0, 1);
  }



  void GridFrame::
  Configure(double position_x,
	    double position_y,
	    double position_theta,
	    double delta)
  {
    _delta = delta;
    _delta_inv = 1 / delta;
    _frame.Set(position_x, position_y, position_theta);
  }



  GridFrame::index_t GridFrame::
  GlobalIndex(position_t point) const
  {
    _frame.From(point.first, point.second);

    return LocalIndex(point);
  }



  GridFrame::index_t GridFrame::
  LocalIndex(position_t point) const
  {
    return index_t((int) rint(point.first  * _delta_inv),
		   (int) rint(point.second * _delta_inv));
  }



  GridFrame::position_t GridFrame::
  GlobalPoint(index_t index) const
  {
    position_t point = LocalPoint(index);
    _frame.To(point.first, point.second);

    return point;
  }



  GridFrame::position_t GridFrame::
  LocalPoint(index_t index) const
  {
    return position_t(index.first * _delta, index.second * _delta);
  }



  /**
     \todo brute force :-(
  */
  void GridFrame::
  SetLocalDisk(GridLayer & grid,
	       position_t center,
	       double radius,
	       double value)
  {
    index_t index = LocalIndex(center);
  
    if(grid.Inside(index))
      grid.Set(index, value);
  
    center.first  -= radius;
    center.second -= radius;
    index_t index_min = LocalIndex(center);

    center.first  += 2 * radius;
    center.second += 2 * radius;
    index_t index_max = LocalIndex(center);

    center.first  -= radius;
    center.second -= radius;
  
    const double radius2(radius * radius);

    index_t i;
    for(i.first = index_min.first;
	i.first < index_max.first;
	++i.first)
      for(i.second = index_min.second;
	  i.second < index_max.second;
	  ++i.second){
	position_t point = LocalPoint(i);
      
	if( ! grid.Inside(i))
	  continue;

	point.first  -= center.first;
	point.second -= center.second;
	double r2 = point.first * point.first + point.second * point.second;

	if(r2 > radius2)
	  continue;

	grid.Set(i, value);
      }
  }



  void GridFrame::
  SetGlobalDisk(GridLayer & grid,
		position_t center,
		double radius,
		double value)
  {
    _frame.From(center.first, center.second);
    SetLocalDisk(grid, center, radius, value);
  }



}
