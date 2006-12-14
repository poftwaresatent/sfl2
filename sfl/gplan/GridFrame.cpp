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
#include <sfl/util/numeric.hpp>
#include <cmath>


namespace sfl {
  
  
  GridFrame::
  GridFrame(double delta)
    : Frame(),
      m_delta(delta),
      m_delta_inv(1 / delta)
  {
  }
  
  
  GridFrame::
  GridFrame(double x, double y, double theta, double delta)
    : Frame(x, y, theta),
      m_delta(delta),
      m_delta_inv(1 / delta)
  {
  }
  
  
  GridFrame::
  GridFrame(const GridFrame & orig)
    : Frame(orig),
      m_delta(orig.m_delta),
      m_delta_inv(orig.m_delta_inv)
  {
  }
  
  
  GridFrame::
  GridFrame(const Frame & frame, double delta)
    : Frame(frame),
      m_delta(delta),
      m_delta_inv(1 / delta)
  {
  }
  
  
  void GridFrame::
  Configure(double position_x, double position_y, double position_theta,
	    double delta)
  {
    m_delta = delta;
    m_delta_inv = 1 / delta;
    Set(position_x, position_y, position_theta);
  }
  
  
  GridFrame::index_t GridFrame::
  GlobalIndex(double px, double py) const
  {
    From(px, py);
    return LocalIndex(px, py);
  }
  
  
  GridFrame::index_t GridFrame::
  GlobalIndex(position_t point) const
  {
    From(point.v0, point.v1);
    return LocalIndex(point);
  }
  
  
  GridFrame::index_t GridFrame::
  LocalIndex(double px, double py) const
  {
    return index_t(static_cast<int>(rint(px * m_delta_inv)),
		   static_cast<int>(rint(py * m_delta_inv)));
  }
  
  
  GridFrame::index_t GridFrame::
  LocalIndex(position_t point) const
  {
    return index_t(static_cast<int>(rint(point.v0 * m_delta_inv)),
		   static_cast<int>(rint(point.v1 * m_delta_inv)));
  }
  
  
  GridFrame::position_t GridFrame::
  GlobalPoint(size_t ix, size_t iy) const
  {
    position_t point(LocalPoint(ix, iy));
    To(point.v0, point.v1);
    return point;
  }
  
  
  GridFrame::position_t GridFrame::
  GlobalPoint(index_t index) const
  {
    position_t point(LocalPoint(index));
    To(point.v0, point.v1);
    return point;
  }
  
  
  GridFrame::position_t GridFrame::
  LocalPoint(size_t ix, size_t iy) const
  {
    return position_t(ix * m_delta, iy * m_delta);
  }
  
  
  GridFrame::position_t GridFrame::
  LocalPoint(index_t index) const
  {
    return position_t(index.v0 * m_delta, index.v1 * m_delta);
  }
  
  
  /** \todo this implementation is a bit brute force... */
  void GridFrame::
  SetLocalDisk(grid_t & grid, position_t center,
	       double radius, double value)
  {
    index_t index(LocalIndex(center));
    if(grid.ValidIndex(index))
      grid[index] = value;
    const index_t minidx(LocalIndex(center.v0 - radius, center.v1 - radius));
    const index_t maxidx(LocalIndex(center.v0 + radius, center.v1 + radius));
    const double radius2(sqr(radius));
    for(size_t ix(minidx.v0); ix < maxidx.v0; ++ix)
      for(size_t iy(minidx.v1); iy < maxidx.v1; ++iy)
	if(grid.ValidIndex(ix, iy)){
	  const position_t point(LocalPoint(ix, iy) - center);
	  const double r2(sqr(point.v0) + sqr(point.v1));
	  if(r2 <= radius2)
	    grid[ix][iy] = value;
	}
  }
  
  
  void GridFrame::
  SetGlobalDisk(grid_t & grid, position_t center,
		double radius, double value)
  {
    From(center.v0, center.v1);
    SetLocalDisk(grid, center, radius, value);
  }
  
  
  size_t GridFrame::
  DrawLocalLine(double x0, double y0, double x1, double y1,
		size_t xsize, size_t ysize, draw_callback & cb)
  {
    x0 /= m_delta;
    y0 /= m_delta;
    x1 /= m_delta;
    y1 /= m_delta;
    
    size_t ix(static_cast<size_t>(rint(x0)));
    size_t iy(static_cast<size_t>(rint(y0)));
    size_t count(0);
    if((ix < xsize) && (iy < ysize)){
      cb(ix, iy);
      ++count;
    }
    
    double dx(x1 - x0);
    double dy(y1 - y0);
    if(absval(dx) > absval(dy)){
      double slope(dy / dx);
      dx = (dx < 0) ? -1 : 1;
      slope *= dx;
      while(absval(x0 - x1) >= 0.5){
	x0 += dx;
	y0 += slope;
	ix = static_cast<size_t>(rint(x0));
	iy = static_cast<size_t>(rint(y0));
	if((ix < xsize) && (iy < ysize)){
	  cb(ix, iy);
	  ++count;
	}
      }
    }
    else{
      double slope(dx / dy);
      dy = (dy < 0) ? -1 : 1;
      slope *= dy;
      while(absval(y0 - y1) >= 0.5){
	y0 += dy;
	x0 += slope;
	ix = static_cast<size_t>(rint(x0));
	iy = static_cast<size_t>(rint(y0));
	if((ix < xsize) && (iy < ysize)){
	  cb(ix, iy);
	  ++count;
	}
      }
    }
    
    return count;
  }
  
  
  size_t GridFrame::
  DrawGlobalLine(double x0, double y0, double x1, double y1,
		 size_t xsize, size_t ysize, draw_callback & cb)
  {
    From(x0, y0);
    From(x1, y1);
    return DrawLocalLine(x0, y0, x1, y1, xsize, ysize, cb);
  }
  
}
