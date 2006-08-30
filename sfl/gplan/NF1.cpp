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
#include "NF1Wave.hpp"
#include <sfl/api/Scan.hpp>
#include <cmath>


namespace sfl {
  
  
  NF1::
  NF1()
    : m_wave(new NF1Wave())
  {
  }
  
  
  void NF1::
  Initialize(boost::shared_ptr<const Scan> scan,
	     double robot_radius,
	     double goal_radius)
  {
    m_grid.Configure(m_grid_dimension, NF1Wave::FREE);
    
    const size_t nscans(scan->data.size());
    for(size_t is(0); is < nscans; ++is)
      if(scan->data[is].rho >= robot_radius){
	const scan_data gdata(scan->data[is]);
	m_frame.SetGlobalDisk(m_grid,
			      position_t(gdata.globx, gdata.globy),
			      robot_radius,
			      NF1Wave::OBSTACLE);
      }
    
    m_frame.SetGlobalDisk(m_grid, m_global_goal, goal_radius, NF1Wave::FREE);
    m_grid.Set(m_goal_index, NF1Wave::GOAL);
    m_frame.SetGlobalDisk(m_grid, m_global_home, robot_radius, NF1Wave::FREE);
  }
  
  
  void NF1::
  Calculate()
  {
    m_wave->Reset();
    m_wave->AddSeed(m_frame.GlobalIndex(m_global_goal));
    m_wave->Propagate(m_grid);
  }
  
  
  bool NF1::
  ResetTrace()
  {
    m_trace = m_home_index;
    if(m_grid.Get(m_trace) == NF1Wave::FREE)
      return false;
    return true;
  }
  
  
  bool NF1::
  GlobalTrace(position_t & point)
  {
    point = m_frame.GlobalPoint(m_trace);
    if(m_grid.Get(m_trace) == NF1Wave::GOAL)
      return false;
    m_trace = m_wave->SmallestNeighbor(m_grid, m_trace);
    return true;
  }
  
  
  void NF1::
  Configure(position_t robot_position,
	    position_t global_goal,
	    double grid_width,
	    int grid_width_dimension)
  {
    m_global_goal = global_goal;
    m_global_home = robot_position;
    
    if(grid_width_dimension % 2 == 0)
      ++grid_width_dimension;
    
    double dx(global_goal.first  - robot_position.first);
    double dy(global_goal.second - robot_position.second);
    Frame frame(robot_position.first,
		robot_position.second,
		atan2(dy, dx));
    
    double width_offset
      = 0.5 * grid_width * (grid_width_dimension - 1)
      / grid_width_dimension;
    double delta = grid_width / grid_width_dimension;
    double xm_frame = - width_offset;
    double ym_frame = - width_offset;
    frame.To(xm_frame, ym_frame);
    
    m_frame.Configure(xm_frame, ym_frame, frame.Theta(), delta);
    
    m_goal_index = m_frame.GlobalIndex(global_goal);
    m_home_index = m_frame.GlobalIndex(m_global_home);
    
    m_grid_dimension.first =
      (int) ceil((sqrt(dx*dx+dy*dy) + grid_width) / delta);
    
    m_grid_dimension.second = grid_width_dimension;
  }
  
}
