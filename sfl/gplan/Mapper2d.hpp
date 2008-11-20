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


namespace sfl {
	
	
	class Scan;
	class GridFrame;
	
  
  class Mapper2d
	{
	public:
		struct travmap_grow_strategy {
			virtual ~travmap_grow_strategy() {}
			
			/**
				 Determine if and how the travmap might need to be grown to
				 accomodate the point (ix, iy), and potentially resize it
				 accordingly.
				 
				 \return true if the travmap was grown.
			*/
			virtual bool operator () (TraversabilityMap & travmap,
																ssize_t ix, ssize_t iy) = 0;
		};

		struct never_grow: public travmap_grow_strategy {
			virtual bool operator () (TraversabilityMap & travmap,
																ssize_t ix, ssize_t iy)  { return false; }
		};

		struct always_grow: public travmap_grow_strategy {
			virtual bool operator () (TraversabilityMap & travmap,
																ssize_t ix, ssize_t iy);
		};
		
		
	private:
		Mapper2d();
		Mapper2d(const Mapper2d &);
		
	protected:
		Mapper2d(double robot_radius,
						 double buffer_zone,
						 double padding_factor,
						 boost::shared_ptr<TraversabilityMap> travmap,
						 boost::shared_ptr<travmap_grow_strategy> grow_strategy,
						 boost::shared_ptr<RWlock> trav_rwlock);
		
  public:
		typedef GridFrame::index_t index_t;
		typedef TraversabilityMap::draw_callback draw_callback;
		
		
		Mapper2d(const GridFrame & gridframe,
						 ssize_t grid_xbegin,
						 ssize_t grid_xend,
						 ssize_t grid_ybegin,
						 ssize_t grid_yend,
						 double robot_radius,
						 double buffer_zone,
						 double padding_factor,
						 int freespace,
						 int obstacle,
						 const std::string & name,
						 boost::shared_ptr<RWlock> trav_rwlock,
						 /** Optional. Defaults to never_grow. */
						 boost::shared_ptr<travmap_grow_strategy> grow_strategy);

		virtual ~Mapper2d() {}
		
		static boost::shared_ptr<Mapper2d>
		Create(double robot_radius,
					 double buffer_zone,
					 double padding_factor,
					 const std::string & traversability_file,
					 /** Optional. Defaults to never_grow. */
					 boost::shared_ptr<travmap_grow_strategy> grow_strategy,
					 std::ostream * err_os);
		
		
		/**
			 Update of traversability map based on a Scan instance, where
			 each scan point is considered an obstacle. We only call
			 TraversabilityMap::SetValue() for actually changed cells, so
			 the draw_callback is only called for those cells that actually
			 get modified. The travmap_grow_strategy registered with the
			 Mapper2d determines if and how the TraversabilityMap gets
			 resized.
			 
			 \return The number of cells that got changed.
		*/
		size_t Update(const Frame & pose, const Scan & scan,
									draw_callback * cb = 0);
		
		/**
			 Similar to Update(const Frame&, const Scan&, draw_callback *),
			 but based on arrays of local (x, y) obstacle point coordinates.
			 
			 \return The number of cells that got changed.
		*/
		size_t Update(const Frame & pose,
									size_t length, double * locx, double * locy,
									draw_callback * cb = 0);
		
		/**
			 Draw a (non-fileld) circle of obstacle points using
			 GridFrame::DrawGlobalCircle(). Each grid cell on the circle is
			 considered a workspace-obstacle, and will be grown by the robot
			 radius and buffer zone.
			 
			 \return The number of cells that got changed.
		*/
		size_t AddObstacleCircle(double globx, double globy, double radius,
														 draw_callback * cb = 0);
		
		boost::shared_ptr<RDTravmap> CreateRDTravmap() const;
		boost::shared_ptr<WRTravmap> CreateWRTravmap();
		
		const GridFrame & GetGridFrame() const { return gridframe; }
		
		
		const int freespace;
		const int obstacle;
		const int ws_obstacle;
		const GridFrame gridframe;
		const double buffer_zone;
		const double grown_safe_distance;
		const double grown_robot_radius;
		
		
		/**
			 In case you want to draw buffered obstacle lines into the map,
			 call GridFrame::DrawGlobalLine() (or DrawLocalLine() or
			 DrawDDALine()) with an instance of buffered_obstacle_adder. The
			 extra Mapper2d::draw_callback constructir argument can be used
			 to pass through another drawer, which will get called for each
			 individual cell in the buffered obstacle region.
		*/
		class buffered_obstacle_adder
			: public GridFrame::draw_callback
		{
		public:
			buffered_obstacle_adder(Mapper2d * _m2d, Mapper2d::draw_callback * _cb);
			virtual void operator () (ssize_t ix, ssize_t iy);
			Mapper2d * m2d;
			Mapper2d::draw_callback * cb;
			size_t count;
		};
		
		/** \return The number of cells that were changed. */
		size_t AddBufferedObstacle(double globx, double globy, draw_callback * cb);

		size_t AddBufferedObstacle(index_t source_index, draw_callback * cb);
		
		
	protected:
		struct sprite_element {
      sprite_element(ssize_t _x, ssize_t _y, int _v): x(_x), y(_y), v(_v) { }
      ssize_t x, y;
      int v;
    };
    
    typedef std::vector<sprite_element> sprite_t;
		
		
		void InitSprite();
		
		/** Default implementation does nothing. Quick hack for
				ReflinkMapper2d rfct. */
		virtual void ResizeNotify(ssize_t grid_xbegin, ssize_t grid_xend,
															ssize_t grid_ybegin, ssize_t grid_yend);
		
		/** Default implementation does nothing. Quick hack for
				ReflinkMapper2d rfct. */		
		virtual bool AddReference(index_t source_index,
															ssize_t target_ix, ssize_t target_iy,
															int value);
		
		/** We use obstacle+1 to keep track of those cells that are non-CS
				expanded obstacles, to distinguish them from obstacle cells
				that do not contain any actual workspace points. */
		boost::shared_ptr<TraversabilityMap> m_travmap;
		boost::shared_ptr<RWlock> m_trav_rwlock;
		sprite_t m_sprite;
		ssize_t m_sprite_x0, m_sprite_y0, m_sprite_x1, m_sprite_y1;	// bbox
		
		boost::shared_ptr<travmap_grow_strategy> m_grow_strategy;
	};
	
	
	/**
		 Experimental way of implementing RemoveBufferedObstacle().
	*/
  class ReflinkMapper2d
		: public Mapper2d
	{
	protected:
		ReflinkMapper2d(double robot_radius,
										double buffer_zone,
										double padding_factor,
										boost::shared_ptr<TraversabilityMap> travmap,
										boost::shared_ptr<travmap_grow_strategy> grow_strategy,
										boost::shared_ptr<RWlock> trav_rwlock);
		
	public:
		typedef std::set<index_t> link_t;
		typedef std::map<index_t, int> fwd_t;
		typedef std::multimap<int, index_t> rev_t;
		struct ref_s {
			fwd_t forward;
			rev_t reverse;
		};
		typedef	flexgrid<link_t> linkmap_t;
		typedef	flexgrid<ref_s> refmap_t;
		
		static boost::shared_ptr<ReflinkMapper2d>
		Create(double robot_radius,
					 double buffer_zone,
					 double padding_factor,
					 const std::string & traversability_file,
					 /** Optional. Defaults to never_grow. */
					 boost::shared_ptr<travmap_grow_strategy> grow_strategy,
					 std::ostream * err_os);
		
		/**
			 Similar to Update(const Frame&, const Scan&, draw_callback*),
			 but also updates the cells along the whole rays of the scan.
			 
			 \return The number of cells that got changed.
		*/
		size_t SwipedUpdate(const Frame & pose,
												const Multiscanner::raw_scan_collection_t & scans,
												draw_callback * cb = 0);
		
		/** \return The number of cells that were changed. */
		size_t RemoveBufferedObstacle(index_t source_index, draw_callback * cb);
		
		const linkmap_t & GetLinkmap() const { return m_linkmap; }
		const refmap_t & GetRefmap() const { return m_refmap; }
		const link_t & GetFreespaceBuffer() const { return m_freespace_buffer; }
		const link_t & GetObstacleBuffer() const { return m_obstacle_buffer; }
		
	protected:
		virtual void ResizeNotify(ssize_t grid_xbegin, ssize_t grid_xend,
															ssize_t grid_ybegin, ssize_t grid_yend);
		
		/** \note This method does not check if an existing link changes
				value, but simply returns false if there already is a link
				from source to target. Changing values of existing links must
				be done by the caller (or add a ModifyReference() method). */
		virtual bool AddReference(index_t source_index,
															ssize_t target_ix, ssize_t target_iy,
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
		
		link_t m_freespace_buffer;
		link_t m_obstacle_buffer;
		linkmap_t m_linkmap;				// lists all targets of a given source
		refmap_t m_refmap;					// maps all sources of a given target		
	};
	
}

#endif // SFL_MAPPER2D_HPP
