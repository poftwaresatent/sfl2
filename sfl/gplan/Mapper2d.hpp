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


#ifndef SFL_MAPPER2D_HPP
#define SFL_MAPPER2D_HPP


#include <sfl/gplan/TraversabilityMap.hpp>
#include <boost/shared_ptr.hpp>
#include <map>


namespace estar {
	class Sprite;
}


namespace sfl {
	
	
	class Scan;
	class TraversabilityMap;
	class GridFrame;
	
  
  class Mapper2d
	{
	private:
		Mapper2d();
		Mapper2d(const Mapper2d &);
		
		Mapper2d(double robot_radius,
						 double buffer_zone,
						 boost::shared_ptr<TraversabilityMap> travmap);
		
  public:
		typedef TraversabilityMap::draw_callback draw_callback;
		
		Mapper2d(const GridFrame & gridframe,
						 size_t grid_ncells_x,
						 size_t grid_ncells_y,
						 double robot_radius,
						 double buffer_zone,
						 int freespace,
						 int obstacle,
						 const std::string & name);
		
		static boost::shared_ptr<Mapper2d>
		Create(double robot_radius, double buffer_zone,
					 const std::string & traversability_file,
					 std::ostream * err_os);
		
		
		/**
			 Update of traversability map based on a Scan instance, where
			 each scan point is considered an obstacle.
			 
			 \note Because we only call TraversabilityMap::SetValue() for
			 actually changed cells, the draw_callback is only called for
			 those cells that actually get modified.
			 
			 \return The number of cells that got changed.
		*/
		size_t Update(const Frame & pose, const Scan & scan,
									draw_callback * cb = 0);
		
		/**
			 Update of traversability map based on arrays of local (x, y)
			 obstacle point coordinates.
			 
			 \note Because we only call TraversabilityMap::SetValue() for
			 actually changed cells, the draw_callback is only called for
			 those cells that actually get modified.
			 
			 \return The number of cells that got changed.
		*/
		size_t Update(const Frame & pose,
									size_t length, double * locx, double * locy,
									draw_callback * cb = 0);
		
		boost::shared_ptr<const TraversabilityMap> GetTravmap() const
		{ return m_travmap; }
		
		
	private:
		/** \return The number of cells that were changed. */
		size_t AddBufferedObstacle(double globx, double globy, draw_callback * cb);
		
		//to do// 		/** \return The number of cells that were changed. */
		//to do// 		size_t SwipeBufferedObstacle(double globx0, double globy0,
		//to do// 																 double globx1, double globy1,
		//to do// 																 draw_callback * cb);
		
		bool AddReference(GridFrame::index_t source_index,
											size_t target_ix, size_t target_iy,
											int value);
		
		//to do///** \return true if the reference existed, in which case the value
		//to do//		will be set to what the influence of source on target used to
		//to do//		be. */
		//to do// 		bool RemoveReference(size_t source_ix, size_t source_iy,
		//to do// 												 size_t target_ix, size_t target_iy,
		//to do// 												 int & value);
		
		
		const ssize_t m_xsize;
		const ssize_t m_ysize;
		const int m_freespace;
		const int m_obstacle;
		const int m_ws_obstacle;
		const GridFrame m_gridframe;
		const double m_buffer_zone;
		const double m_grown_safe_distance;
		const double m_grown_robot_radius;
		
		/** We use obstacle+1 to keep track of those cells that are non-CS
				expanded obstacles, to distinguish them from obstacle cells
				that do not contain any actual workspace points. */
		boost::shared_ptr<TraversabilityMap> m_travmap;
		boost::shared_ptr<estar::Sprite> m_sprite;
		
		
		typedef std::map<GridFrame::index_t, int> fwd_map_t;
		typedef std::multimap<int, GridFrame::index_t> rev_map_t;
		
		struct ref_s {
			fwd_map_t forward;
			rev_map_t reverse;
		};
		
		typedef	array2d<ref_s> refmap_t;
		
		refmap_t m_refmap;
	};
	
}

#endif // SFL_MAPPER2D_HPP
