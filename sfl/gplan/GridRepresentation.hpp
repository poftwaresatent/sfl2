/* -*- mode: C++; tab-width: 2 -*- */
/* 
 * Copyright (C) 2009 Roland Philippsen <roland dot philippsen at gmx dot net>
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

#ifndef SUNFLOWER_GRID_REPRESENTATION_HPP
#define SUNFLOWER_GRID_REPRESENTATION_HPP

#include <sfl/util/flexgrid.hpp>
#include <sfl/gplan/GridFrame.hpp>

namespace sfl {
	
	
	/**
		 Provides an easy way to manage data that is stored in a 2D grid
		 that can be arbitrarily positioned in the
		 environment. GridRepresentation wraps a flexgrid instance, adding
		 functions for growing the grid on the fly, and transforming from
		 global coordinates to grid indices and vice-versa.
	 */
  template<typename value_type>
  class GridRepresentation
  {
  public:
		typedef value_type value_t;
		typedef flexgrid<value_t> grid_t;
		typedef flexgrid<value_t const> const const_grid_t;
		
		GridFrame gframe;
		grid_t grid;
		
		
		/** Default constructor: empty grid positioned at (0, 0, 0) with
				resolution 1. */
    GridRepresentation()
			: gframe(0, 0, 0, 1)
		{}
		
		/** Constructor with origin and grid size. Uses a
				default-constructed value for initialization of the grid. */
		GridRepresentation(const GridFrame & origin,
											 ssize_t xbegin, ssize_t xend,
											 ssize_t ybegin, ssize_t yend)
			: gframe(origin)
		{ grid.resize(xbegin, xend, ybegin, yend); }
		
		/** Constructor with origin, grid size, and initial value. */
		GridRepresentation(const GridFrame & origin,
											 ssize_t xbegin, ssize_t xend,
											 ssize_t ybegin, ssize_t yend,
											 value_t const & initval)
			: gframe(origin)
		{ grid.resize(xbegin, xend, ybegin, yend, initval); }
		
		
		/** Fill all cells with the given value. */
		void Reset(value_t const & value)	{
			for (typename grid_t::iterator ii(grid.begin()); ii != grid.end(); ++ii)
				*ii = value;
		}
		
		/** \return true if the given index lies within the grid. */
		bool IsValidIdx(ssize_t index_x, ssize_t index_y) const
		{ return grid.valid(index_x, index_y); }
		
		/** Like IsValidIndex(ssize_t, ssize_t) but operates on global
				coordinates that first get transformed into the GridFrame. */
		bool IsValidCoord(double global_x, double global_y) const {
			const GridFrame::index_t idx(gframe.GlobalIndex(global_x, global_y));
			return grid.valid(idx.v0, idx.v1);
		}
		
		/** Make sure that the given index lies within the grid, if
				necessary expanding the underlying flexgrid and setting any
				new cells to the given fill_value.
				
				\return true if it was necessary to add cells. */
		bool AutogrowIdx(ssize_t index_x, ssize_t index_y, value_t const & fill_value) {
			if (grid.valid(index_x, index_y))
				return false;
			if (index_x < grid.xbegin())
				grid.resize_xbegin(index_x, fill_value);
			if (index_x >= grid.xend())
				grid.resize_xend(index_x + 1, fill_value);
			if (index_y < grid.ybegin())
				grid.resize_ybegin(index_y, fill_value);
			if (index_y >= grid.yend())
				grid.resize_yend(index_y + 1, fill_value);
			return true;
		}
		
		/** Like Autogrow(ssize_t, ssize_t, value_t const &) but operates
				on global coordinates that first get transformed into the
				GridFrame. */
		bool AutogrowCoord(double global_x, double global_y, value_t const & fill_value) {
			const GridFrame::index_t idx(gframe.GlobalIndex(global_x, global_y));
			return AutogrowIdx(idx.v0, idx.v1, fill_value);
		}
		
    /** \return if the index is in the grid, in which case the
				out-parameter <code>value</code> is set to it. */
		bool GetValueIdx(ssize_t index_x, ssize_t index_y, value_t & value) const {
			if( ! grid.valid(index_x, index_y))
				return false;
			value = grid.at(index_x, index_y);
			return true;
		}
		
    /** \return true if the global coordinates is in the grid, in
				which case the out-parameter <code>value</code> is set to
				it. */
		bool GetValueCoord(double global_x, double global_y, value_t & value) const {
			const GridFrame::index_t idx(gframe.GlobalIndex(global_x, global_y));
			if( ! grid.valid(idx.v0, idx.v1))
				return false;
			value = grid.at(idx.v0, idx.v1);
			return true;
		}
		
    /** \return if the index is in the grid, in which case the given
				<code>value</code> will be stored there. */
		bool SetValueIdx(ssize_t index_x, ssize_t index_y, value_t const & value) {
			if( ! grid.valid(index_x, index_y))
				return false;
			grid.at(index_x, index_y) = value;
			return true;
		}
		
    /** \return if the global coordinate is in the grid, in which case
				the given <code>value</code> will be stored there. */
		bool SetValueCoord(double global_x, double global_y, value_t const & value) {
			const GridFrame::index_t idx(gframe.GlobalIndex(global_x, global_y));
			return SetValueIdx(idx.v0, idx.v1, value);
		}
		
		ssize_t GetXBegin() const { return grid.xbegin(); }
		ssize_t GetXEnd()   const { return grid.xend(); }
		ssize_t GetYBegin() const { return grid.ybegin(); }
		ssize_t GetYEnd()   const { return grid.yend(); }
  };
  
}

#endif // SUNFLOWER_GRID_REPRESENTATION_HPP
