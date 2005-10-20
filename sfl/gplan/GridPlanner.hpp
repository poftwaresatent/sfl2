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


#ifndef SUNFLOWER_GRIDPLANNER_HPP
#define SUNFLOWER_GRIDPLANNER_HPP



#include <sfl/api/GlobalScan.hpp>
#include <sfl/gplan/GridFrame.hpp>



namespace sfl {



  class GridPlanner
  {
  public:
    typedef GridFrame::index_t index_t;
    typedef GridFrame::position_t position_t;



    static const double FREE     = -2;
    static const double OBSTACLE = -1;
    static const double GOAL     =  0;



    virtual ~GridPlanner();

    void Configure(position_t robot_position,
		   position_t global_goal,
		   double grid_width,
		   int grid_width_dimension);
  
    /** \note The GlobalScan object should be filtered, ie contain
	only valid readings. This can be obtained from
	Multiscanner::CollectScans() and
	Multiscanner::CollectGlobalScans(), whereas
	Scanner::GetScanCopy() can still contain readings that are out
	of range (represented as readings at the maximum rho
	value). */
    virtual void Initialize(boost::shared_ptr<const GlobalScan> scan,
			    double robot_radius,
			    double goal_radius) = 0;
  
    virtual void Calculate() = 0;
  
    /** \return false if unreachable */
    virtual bool ResetTrace() = 0;
  
    /** \return false if at goal (end of trace) */
    virtual bool GlobalTrace(position_t & point) = 0;

    /** \note just needed for e.g. plotting of internal data. */
    inline const GridFrame & GetGridFrame() const;
  

  protected:
    GridFrame _frame;
    index_t _grid_dimension;
    position_t _global_goal;
    index_t _goal_index;
    position_t _global_home;
    index_t _home_index;
  };


  const GridFrame & GridPlanner::
  GetGridFrame()
    const
  {
    return _frame;
  }

}

#endif // SUNFLOWER_GRIDPLANNER_HPP
