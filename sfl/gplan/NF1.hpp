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


#ifndef SUNFLOWER_NF1_H
#define SUNFLOWER_NF1_H


#include <sfl/gplan/GridLayer.hpp>
#include <sfl/gplan/GridFrame.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/scoped_ptr.hpp>


namespace sfl {
  
  
  class Scan;
  class NF1Wave;
  
  
  class NF1
  {
  public:
    typedef GridFrame::index_t index_t;
    typedef GridFrame::position_t position_t;
    
    NF1();
    
    void Configure(position_t robot_position,
		   position_t global_goal,
		   double grid_width,
		   int grid_width_dimension);
    
    /** \note The Scan object should be filtered, ie contain only
	valid readings. This can be obtained from
	Multiscanner::CollectScans(), whereas Scanner::GetScanCopy()
	can still contain readings that are out of range (represented
	as readings at the maximum rho value). */
    void Initialize(boost::shared_ptr<const Scan> scan,
		    double robot_radius,
		    double goal_radius);
    
    void Calculate();
    bool ResetTrace();
    bool GlobalTrace(position_t & point);
    
    /** \note Only needed for plotting. */
    const GridLayer & GetGridLayer() const { return m_grid; }
    
    /** \note Only needed for plotting. */
    const GridFrame & GetGridFrame() const { return m_frame; }
    
    
  private:
    GridFrame m_frame;
    index_t m_grid_dimension;
    position_t m_global_goal;
    index_t m_goal_index;
    position_t m_global_home;
    index_t m_home_index;
    GridLayer m_grid;
    boost::scoped_ptr<NF1Wave> m_wave;
    index_t m_trace;
  };
  
}

#endif // SUNFLOWER_NF1_H
