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


#ifndef SUNFLOWER_GRIDFRAME_HPP
#define SUNFLOWER_GRIDFRAME_HPP



#include <sfl/oa/gplan/GridLayer.hpp>
#include <sfl/util/Frame.hpp>
#include <utility>



namespace sfl {



  class GridFrame
  {
  public:
    typedef GridLayer::index_t index_t;
    typedef std::pair<double, double> position_t;


    GridFrame();

    void Configure(double position_x,
		   double position_y,
		   double position_theta,
		   double delta);

    index_t GlobalIndex(position_t point) const;
    index_t LocalIndex(position_t point) const;

    position_t GlobalPoint(index_t index) const;
    position_t LocalPoint(index_t index) const;

    void SetLocalDisk(GridLayer & grid,
		      position_t center,
		      double radius,
		      double value);

    void SetGlobalDisk(GridLayer & grid,
		       position_t center,
		       double radius,
		       double value);



  private:
    double _delta;
    double _delta_inv;

    Frame _frame;
  };



}

#endif // SUNFLOWER_GRIDFRAME_HPP
