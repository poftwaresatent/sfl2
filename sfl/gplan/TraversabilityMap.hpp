/* -*- mode: C++; tab-width: 2 -*- */
/* 
 * Copyright (C) 2006
 * Swiss Federal Institute of Technology, Zurich. All rights reserved.
 * 
 * Developed at the Autonomous Systems Lab.
 * Visit our homepage at http://www.asl.ethz.ch/
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


#ifndef SUNFLOWER_TRAVERSABILITY_MAP_HPP
#define SUNFLOWER_TRAVERSABILITY_MAP_HPP


#include <sfl/util/flexgrid.hpp>
#include <sfl/gplan/GridFrame.hpp>
#include <sfl/util/Frame.hpp>
#include <boost/shared_ptr.hpp>
#include <string>
#include <iosfwd>


namespace sfl {
  
  
  class TraversabilityMap
  {
  public:
		/** \todo rename to change_notify or so... "draw_callback" is an
				anachronism that doesn't reflect its current use. */
    struct draw_callback {
      virtual ~draw_callback() {}
			/** This is called BEFORE the new value is stored in the
					traversability map. It is ALSO called if
					oldval==newval. It's up to subclasses to (maybe) skip
					computations when nothing changed. */
      virtual void operator () (ssize_t ix, ssize_t iy,
																int oldval, int newval) = 0;
    };
		
		/** Debug utility that simply prints out the parameters passed to
				it, with one leading whitespace. */
		struct dbg_draw_callback: public draw_callback {
			dbg_draw_callback(std::ostream & os);
      virtual void operator () (ssize_t ix, ssize_t iy,
																int oldval, int newval);
			std::ostream & m_os;
		};
		
		
    TraversabilityMap();
		
		/** \note Grid range ends are non-inclusive (xend and yend are
				"one past" the end). */
		TraversabilityMap(const GridFrame & origin,
											ssize_t xbegin, ssize_t xend,
											ssize_t ybegin, ssize_t yend);
		
		/**
			 \note It is not recommended to use <code>obstacle <=
			 freespace</code> as several clients implicitly assume
			 <code>obstacle > freespace</code>! Also, grid range ends are
			 non-inclusive (xend and yend are "one past" the end).
		*/
		TraversabilityMap(const GridFrame & origin,
											ssize_t xbegin, ssize_t xend,
											ssize_t ybegin, ssize_t yend,
											int freespace, int obstacle,
											const std::string & name);
		
    static boost::shared_ptr<TraversabilityMap>
    Parse(std::istream & is, std::ostream * os);
    
		
		/** \return true if the given index lies within the grid. */
		bool IsValid(ssize_t index_x, ssize_t index_y) const;
		
		/** Like IsValid(ssize_t, ssize_t) but operates on global
				coordinates that first get transformed into the GridFrame. */
		bool IsValid(double global_x, double global_y) const;
		
		/**
			 Make sure that the given index lies within the grid, if
			 necessary expanding the underlying flexgrid and setting any new
			 cells to the given fill_value.

			 \return true if it was necessary to add cells.
		*/
		bool Autogrow(ssize_t index_x, ssize_t index_y, int fill_value);
		
		/** Like Autogrow(ssize_t, ssize_t, int, grow_notify*) but
				operates on global coordinates that first get transformed into
				the GridFrame. */
		bool Autogrow(double global_x, double global_y, int fill_value);
		
    /**
       \return true if the traversability at the given global
       coordinates is known, in which case the out-parameter
       <code>value</code> is set to it.
    */
		bool GetValue(double global_x, double global_y, int & value) const;
		
		bool GetValue(ssize_t index_x, ssize_t index_y, int & value) const;
		
		bool GetValue(size_t index_x, size_t index_y, int & value) const
		{ return GetValue(static_cast<ssize_t>(index_x),
											static_cast<ssize_t>(index_y),
											value); }
		
		bool SetValue(double global_x, double global_y, int value,
									draw_callback * cb);
		
		bool SetValue(ssize_t index_x, ssize_t index_y, int value,
									draw_callback * cb);
		
		bool SetValue(size_t index_x, size_t index_y, int value,
									draw_callback * cb)
		{ return SetValue(static_cast<ssize_t>(index_x),
											static_cast<ssize_t>(index_y),
											value, cb); }

		bool SetObst(double global_x, double global_y, draw_callback * cb);
		bool SetObst(ssize_t index_x, ssize_t index_y, draw_callback * cb);
		bool SetFree(double global_x, double global_y, draw_callback * cb);
		bool SetFree(ssize_t index_x, ssize_t index_y, draw_callback * cb);
		
		/** \return true if the given point lies in the grid and there is
				an obstacle there. */
		bool IsObst(double global_x, double global_y) const;

		/** \return true if the given index lies in the grid and there is
				an obstacle there. */
		bool IsObst(ssize_t index_x, ssize_t index_y) const;

		/** \return true if the given point lies in the grid and the
				corresponding cell is in free-space. */
		bool IsFree(double global_x, double global_y) const;

		/** \return true if the given index lies in the grid and the
				corresponding cell is in free-space. */
		bool IsFree(ssize_t index_x, ssize_t index_y) const;
		
		void DumpMap(std::ostream * os) const;
    
		
    GridFrame gframe;						/**< default (0, 0, 0, 1) */
    int freespace;							/**< default 0 */
    int obstacle;								/**< default 127 */
    std::string name;						/**< default "world" */
		
		typedef flexgrid<int> grid_t;
		typedef flexgrid<int const> const const_grid_t;
		
		grid_t grid;
  };
  
}

#endif // SUNFLOWER_TRAVERSABILITY_MAP_HPP
