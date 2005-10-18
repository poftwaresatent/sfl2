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


#include "NF1.hpp"



namespace sfl {



  NF1::NF1()
  {
  }



  void NF1::
  Initialize(boost::shared_ptr<const GlobalScan> scan,
	     double robot_radius,
	     double goal_radius)
  {
    _grid.Configure(_grid_dimension, FREE);
  
    const GlobalScan::array_t & gdata(scan->data);
    const Scan::array_t & ldata(scan->local_scan->data);
    const size_t nscans(ldata.size());

    for(size_t is(0); is < nscans; ++is)
      if(ldata[is].rho >= robot_radius)
	_frame.SetGlobalDisk(_grid,
			     position_t(gdata[is].globx, gdata[is].globy),
			     robot_radius,
			     OBSTACLE);
    
    _frame.SetGlobalDisk(_grid,
			 _global_goal,
			 goal_radius,
			 FREE);
    
    _grid.Set(_goal_index, GOAL);
    
    _frame.SetGlobalDisk(_grid,
			 _global_home,
			 robot_radius,
			 FREE);
  }
  
  

  void NF1::
  Calculate()
  {
    _wave.Reset();
    _wave.AddSeed(_frame.GlobalIndex(_global_goal));
    _wave.Propagate(_grid);
  }



  bool NF1::
  ResetTrace()
  {
    _trace = _home_index;

    if(_grid.Get(_trace) == FREE){
      return false;
    }
    
    return true;
  }



  bool NF1::
  GlobalTrace(position_t & point)
  {
    point = _frame.GlobalPoint(_trace);

    if(_grid.Get(_trace) == GOAL){
      return false;
    }

    _trace = _wave.SmallestNeighbor(_grid, _trace);

    return true;
  }



}
