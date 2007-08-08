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


#include <sfl/gplan/RWTravmap.hpp>
#include <sfl/api/Multiscanner.hpp>
#include <map>
#include <set>


namespace estar {
	class Sprite;
}


namespace sfl {
	
	
	class Scan;
	class GridFrame;
	
  
  class Mapper2d
	{
	private:
		Mapper2d();
		Mapper2d(const Mapper2d &);
		
		Mapper2d(double robot_radius,
						 double buffer_zone,
						 boost::shared_ptr<TraversabilityMap> travmap,
						 boost::shared_ptr<RWlock> trav_rwlock);
		
  public:
		typedef GridFrame::index_t index_t;
		typedef TraversabilityMap::draw_callback draw_callback;
		typedef std::set<index_t> link_t;
		typedef std::map<index_t, int> fwd_t;
		typedef std::multimap<int, index_t> rev_t;
		struct ref_s {
			fwd_t forward;
			rev_t reverse;
		};
		typedef	array2d<link_t> linkmap_t;
		typedef	array2d<ref_s> refmap_t;
		
		
		Mapper2d(const GridFrame & gridframe,
						 size_t grid_ncells_x,
						 size_t grid_ncells_y,
						 double robot_radius,
						 double buffer_zone,
						 int freespace,
						 int obstacle,
						 const std::string & name,
						 boost::shared_ptr<RWlock> trav_rwlock);
		
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
			 \return The number of cells that got changed.
		*/
		size_t SwipedUpdate(const Frame & pose,
												const Multiscanner::raw_scan_collection_t & scans,
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
		
		boost::shared_ptr<RDTravmap> CreateRDTravmap() const;
		boost::shared_ptr<WRTravmap> CreateWRTravmap();
		
		const link_t & GetFreespaceBuffer() const { return m_freespace_buffer; }
		const link_t & GetObstacleBuffer() const { return m_obstacle_buffer; }
		const GridFrame & GetGridFrame() const { return gridframe; }
		const linkmap_t & GetLinkmap() const { return m_linkmap; }
		const refmap_t & GetRefmap() const { return m_refmap; }
		
		
		const ssize_t xsize;
		const ssize_t ysize;
		const int freespace;
		const int obstacle;
		const int ws_obstacle;
		const GridFrame gridframe;
		const double buffer_zone;
		const double grown_safe_distance;
		const double grown_robot_radius;
		
		
	private:
		/** \return The number of cells that were changed. */
		size_t AddBufferedObstacle(double globx, double globy, draw_callback * cb);

		size_t AddBufferedObstacle(index_t source_index, draw_callback * cb);
		
		/** \return The number of cells that were changed. */
		size_t RemoveBufferedObstacle(index_t source_index, draw_callback * cb);
		
		/** \note This method does not check if an existing link changes
				value, but simply returns false if there already is a link
				from source to target. Changing values of existing links must
				be done by the caller (or add a ModifyReference() method). */
		bool AddReference(index_t source_index,
											size_t target_ix, size_t target_iy,
											int value);
		
		/**
			 \note This method does NOT remove the link from m_linkmap, as
			 that would make it impossible to loop over targets using an
			 iterator. Callers must ensure to remove the link from m_linkmap
			 after having looped over the targets.
			 
			 \return true if the reference existed, in which case the
			 influence of the source on the target as well as the new target
			 value will be set (the new value can be identical to the old in
			 cases where the influence of this source is overridden by or
			 equal to another source on the same target).
		*/
		bool RemoveReference(index_t source_index,
												 index_t target_index,
												 int & influence, int & new_value);
		
		
		/** We use obstacle+1 to keep track of those cells that are non-CS
				expanded obstacles, to distinguish them from obstacle cells
				that do not contain any actual workspace points. */
		boost::shared_ptr<TraversabilityMap> m_travmap;
		boost::shared_ptr<RWlock> m_trav_rwlock;
		boost::shared_ptr<estar::Sprite> m_sprite;
		
		linkmap_t m_linkmap;				// lists all targets of a given source
		refmap_t m_refmap;					// maps all sources of a given target
		link_t m_freespace_buffer;
		link_t m_obstacle_buffer;
	};
	
}

#endif // SFL_MAPPER2D_HPP
