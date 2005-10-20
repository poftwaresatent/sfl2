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


#include <sfl/gplan/GridPlanner.hpp>
#include <sfl/gplan/GridLayer.hpp>
#include <sfl/gplan/NF1Wave.hpp>
#include <sfl/api/GlobalScan.hpp>


namespace sfl {


  class NF1:
    public GridPlanner
  {
  public:
    NF1();
    
    /** \note The GlobalScan object should be filtered, ie contain
	only valid readings. This can be obtained from
	Multiscanner::CollectScans() and
	Multiscanner::CollectGlobalScans(), whereas
	Scanner::GetScanCopy() can still contain readings that are out
	of range (represented as readings at the maximum rho
	value). */
    void Initialize(boost::shared_ptr<const GlobalScan> scan,
		    double robot_radius,
		    double goal_radius);
  
    void Calculate();
    bool ResetTrace();
    bool GlobalTrace(position_t & point);

    /** \todo Only needed for plotting, should be hidden. */
    inline const GridLayer & GetGridLayer() const;


  private:
    GridLayer _grid;
    NF1Wave _wave;
    index_t _trace;
  };


  const GridLayer & NF1::
  GetGridLayer()
    const
  {
    return _grid;
  }

}

#endif // SUNFLOWER_NF1_H
