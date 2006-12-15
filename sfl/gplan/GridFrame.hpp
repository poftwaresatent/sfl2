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


#include <sfl/util/Frame.hpp>
#include <sfl/util/array2d.hpp>


namespace sfl {
  
  
  class GridFrame
    : public Frame
  {
  public:
    typedef vec2d<size_t> index_t;
    typedef vec2d<double> position_t;
    typedef array2d<double> grid_t;
    
    struct draw_callback {
      virtual ~draw_callback() {}
      virtual void operator () (size_t ix, size_t iy) = 0;
    };
    
    
    explicit GridFrame(double delta);
    GridFrame(double x, double y, double theta, double delta);
    GridFrame(const GridFrame & orig);
    GridFrame(const Frame & frame, double delta);
    
    void Configure(double position_x, double position_y, double position_theta,
		   double delta);
    
    index_t GlobalIndex(double px, double py) const;
    index_t GlobalIndex(position_t point) const;
    index_t LocalIndex(double px, double py) const;
    index_t LocalIndex(position_t point) const;
    
    position_t GlobalPoint(size_t ix, size_t iy) const;
    position_t GlobalPoint(index_t index) const;
    position_t LocalPoint(size_t ix, size_t iy) const;
    position_t LocalPoint(index_t index) const;
    
    void SetLocalDisk(grid_t & grid, position_t center,
		      double radius, double value);
    
    void SetGlobalDisk(grid_t & grid, position_t center,
		       double radius, double value);
    
    /**
       Calls the provided callback functor for each index that lies on
       the line.
       
       \note Uses a Differential Analyzer style algorithm.
       
       \return The number of grid cells drawn.
    */
    size_t DrawLocalLine(double x0, double y0, double x1, double y1,
			 size_t xsize, size_t ysize,
			 draw_callback & cb) const;
    
    /**
       Same as DrawLocalLine() but first transforms the given
       endpoints from the global to the local frame of reference.
    */
    size_t DrawGlobalLine(double x0, double y0, double x1, double y1,
			  size_t xsize, size_t ysize, draw_callback & cb);
    
    double Delta() const { return m_delta; }
    
  protected:
    double m_delta;
    double m_delta_inv;
  };
  
}

#endif // SUNFLOWER_GRIDFRAME_HPP
