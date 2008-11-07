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


#include "Mapper2d.hpp"
#include "GridFrame.hpp"
#include <sfl/util/pdebug.hpp>
#include <sfl/api/Scan.hpp>
#include <estar/Sprite.hpp>
#include <iostream>
#include <fstream>
#include <cmath>
#include <limits>


#ifdef SFL_DEBUG
# define PDEBUG PDEBUG_ERR
#else // ! SFL_DEBUG
# define PDEBUG PDEBUG_OFF
#endif // SFL_DEBUG
#define PVDEBUG PDEBUG_OFF


using namespace std;
using namespace estar;
using namespace boost;


//static const double sqrt_of_two(1.41421356237);
static const double sqrt_of_half(0.707106781187);

namespace local {

	struct swipe_cb
		: public sfl::GridFrame::draw_callback
	{
		typedef sfl::Mapper2d::link_t link_t;
		
		link_t & buffer;
		const sfl::TraversabilityMap & travmap;
		const int ws_obst;
		
		swipe_cb(link_t & _buffer, const sfl::TraversabilityMap & _travmap,
						 int _ws_obst)
			: buffer(_buffer), travmap(_travmap), ws_obst(_ws_obst)
		{
		}

		virtual void operator () (ssize_t ix, ssize_t iy)
		{
			int val;
			if(travmap.GetValue(ix, iy, val) && (ws_obst == val))
				buffer.insert(sfl::Mapper2d::index_t(ix, iy));
		}
		
	};
	
}

using namespace local;


namespace sfl {
	
	
	Mapper2d::
	Mapper2d(const GridFrame & _gridframe,
					 ssize_t grid_xbegin,
					 ssize_t grid_xend,
					 ssize_t grid_ybegin,
					 ssize_t grid_yend,
					 double robot_radius,
					 double _buffer_zone,
					 double padding_factor,
					 int _freespace,
					 int _obstacle,
					 const std::string & name,
					 shared_ptr<RWlock> trav_rwlock,
					 shared_ptr<travmap_grow_strategy> grow_strategy)
		: freespace(_freespace),
			obstacle(_obstacle),
			ws_obstacle(_obstacle + 1),
			gridframe(_gridframe),
			buffer_zone(_buffer_zone),
			grown_safe_distance(robot_radius + _buffer_zone
													+ padding_factor * _gridframe.Delta() * sqrt_of_half),
			grown_robot_radius(robot_radius
												 + padding_factor * _gridframe.Delta() * sqrt_of_half),
			m_travmap(new TraversabilityMap(_gridframe,
																			grid_xbegin, grid_xend,
																			grid_ybegin, grid_yend,
																			freespace, obstacle, name)),
			m_trav_rwlock(trav_rwlock),
			m_sprite(new Sprite(grown_safe_distance, _gridframe.Delta()))
	{
		m_linkmap.resize(grid_xbegin, grid_xend, grid_ybegin, grid_yend);
		m_refmap.resize(grid_xbegin, grid_xend, grid_ybegin, grid_yend);
		
		if (grow_strategy)
			m_grow_strategy = grow_strategy;
		else
			m_grow_strategy.reset(new never_grow());			
	}
	
	
	Mapper2d::
	Mapper2d(double robot_radius,
					 double _buffer_zone,
					 double padding_factor,
					 shared_ptr<TraversabilityMap> travmap,
					 shared_ptr<travmap_grow_strategy> grow_strategy,
					 shared_ptr<RWlock> trav_rwlock)
		: freespace(travmap->freespace),
			obstacle(travmap->obstacle),
			ws_obstacle(travmap->obstacle + 1),
			gridframe(travmap->gframe),
			buffer_zone(_buffer_zone),
			grown_safe_distance(robot_radius + _buffer_zone
													+ padding_factor * gridframe.Delta() * sqrt_of_half),
			grown_robot_radius(robot_radius
												 + padding_factor * gridframe.Delta() * sqrt_of_half),
			m_travmap(travmap),
			m_trav_rwlock(trav_rwlock),
			m_sprite(new Sprite(grown_safe_distance, gridframe.Delta()))
	{
		ssize_t const grid_xbegin(travmap->grid.xbegin());
		ssize_t const grid_xend(travmap->grid.xend());
		ssize_t const grid_ybegin(travmap->grid.ybegin());
		ssize_t const grid_yend(travmap->grid.yend());
		m_linkmap.resize(grid_xbegin, grid_xend, grid_ybegin, grid_yend);
		m_refmap.resize(grid_xbegin, grid_xend, grid_ybegin, grid_yend);
		
		if (grow_strategy)
			m_grow_strategy = grow_strategy;
		else
			m_grow_strategy.reset(new never_grow());			
	}
	
	
	shared_ptr<Mapper2d> Mapper2d::
	Create(double robot_radius,
				 double buffer_zone,
				 double padding_factor,
				 const std::string & traversability_file,
				 boost::shared_ptr<travmap_grow_strategy> grow_strategy,
				 std::ostream * err_os)
	{
		shared_ptr<RWlock> rwl(RWlock::Create("sfl::Mapper2d::m_trav_rwlock"));
		if( ! rwl){
      if(err_os) *err_os << "ERROR in Mapper2d::Create():\n"
												 << "  sfl::RWlock::Create() failed\n";
      return shared_ptr<Mapper2d>();
    }
		
		ifstream trav(traversability_file.c_str());
    if( ! trav){
      if(err_os) *err_os << "ERROR in Mapper2d::Create():\n"
												 << "  invalid traversability file \""
												 << traversability_file << "\".\n";
      return shared_ptr<Mapper2d>();
    }
    shared_ptr<TraversabilityMap>
      traversability(TraversabilityMap::Parse(trav, err_os));
    if( ! traversability){
      if(err_os) *err_os << "ERROR in Mapper2d::Create():\n"
												 << "  TraversabilityMap::Parse() failed on \""
												 << traversability_file << "\".\n";
      return shared_ptr<Mapper2d>();
    }
		
		shared_ptr<Mapper2d>
			result(new Mapper2d(robot_radius, buffer_zone, padding_factor, traversability,
													grow_strategy, rwl));
		return result;
	}
	
	
	size_t Mapper2d::
	Update(const Frame & pose, size_t length, double * locx, double * locy,
				 draw_callback * cb)
	{
		RWlock::wrsentry sentry(m_trav_rwlock);
		size_t count(0);
		for(size_t ii(0); ii < length; ++ii){
			double xw(locx[ii]);
			double yw(locy[ii]);
			pose.To(xw, yw);
			count += AddBufferedObstacle(xw, yw, cb);
		}
		return count;
	}
	
	
	size_t Mapper2d::
	Update(const Frame & pose, const Scan & scan,
				 draw_callback * cb)
	{
		RWlock::wrsentry sentry(m_trav_rwlock);
		const Scan::array_t & scan_data(scan.data);
		size_t count(0);		
		for(size_t ii(0); ii < scan_data.size(); ++ii)
			count +=
				AddBufferedObstacle(scan_data[ii].globx, scan_data[ii].globy, cb);
		return count;
	}
	
	
	size_t Mapper2d::
	AddBufferedObstacle(double globx, double globy,
											draw_callback * cb)
	{
		return AddBufferedObstacle(gridframe.GlobalIndex(globx, globy), cb);
	}
	
	
	size_t Mapper2d::
	AddBufferedObstacle(index_t source_index,
											draw_callback * cb)
	{
		ssize_t const ix0(source_index.v0);
		ssize_t const iy0(source_index.v1);
		
		PVDEBUG("source index %lu %lu\n", ix0, iy0);
		
		// Check if the cell is in the grid or we already have a "true"
		// (workspace) obstacle here, in which case we can skip the rest
		int old_value;
		if (m_travmap->GetValue(ix0, iy0, old_value)
				&& (ws_obstacle == old_value))
			return 0;
		
		// See if we (might) need to grow the grid...
		ssize_t bbx0, bby0, bbx1, bby1;
		m_sprite->GetBBox(bbx0, bby0, bbx1, bby1);
		bbx0 += ix0;
		bbx1 += ix0;
		bby0 += iy0;
		bby1 += iy0;
		bool grown(false);
		grown |= (*m_grow_strategy)(*m_travmap, bbx0, bby0);
		grown |= (*m_grow_strategy)(*m_travmap, bbx0, bby1);
		grown |= (*m_grow_strategy)(*m_travmap, bbx1, bby0);
		grown |= (*m_grow_strategy)(*m_travmap, bbx1, bby1);
		if (grown) {
			ssize_t const xbegin(m_travmap->grid.xbegin());
			ssize_t const xend(m_travmap->grid.xend());
			ssize_t const ybegin(m_travmap->grid.ybegin());
			ssize_t const yend(m_travmap->grid.yend());
			PDEBUG("grown to range (%d, %d, %d, %d) for bbox (%d, %d, %d, %d)\n",
						 xbegin, xend, ybegin, yend,
						 bbx0, bby0, bbx1, bby1);
			m_linkmap.resize(xbegin, xend, ybegin, yend);
			m_refmap.resize(xbegin, xend, ybegin, yend);
		}
		
		// Ready to insert W-space obstacle and perform C-space expansion
		m_travmap->SetValue(ix0, iy0, ws_obstacle, cb);
		size_t count(1);
		
		PVDEBUG("set source to %d\n", ws_obstacle);

		// Perform C-space extension with buffer zone, housekeeping the
		// cell-dependency structure.
    const Sprite::indexlist_t & area(m_sprite->GetArea());
		for (size_t ii(0); ii < area.size(); ++ii) {
			ssize_t const ix(ix0 + area[ii].x);
			ssize_t const iy(iy0 + area[ii].y);
			if ( ! m_travmap->IsValid(ix, iy))
				continue;								// outside of grid
			if ((ix0 == ix) && (iy0 == iy))
				continue;								// we've already flagged the center
			
			double const dist(area[ii].r);
			if (dist > grown_safe_distance)
				continue;								// outside buffer ("never" happens)
			
			int value(obstacle);
			if (dist > grown_robot_radius) {
				double const vv(  freespace * (dist - grown_robot_radius)
												+ obstacle  * (grown_safe_distance - dist));
				value = static_cast<int>(rint(vv / buffer_zone));
			}
			
			PVDEBUG("target %lu %lu to %d\n", ix, iy, value);
			
			AddReference(source_index, ix, iy, value);
			m_travmap->GetValue(ix, iy, old_value);
			if (value > old_value) {
				PVDEBUG("value = %d > old_value = %d\n", value, old_value);
				m_travmap->SetValue(ix, iy, value, cb);
				++count;
			}
		}
		
		PVDEBUG("changed %lu cells\n", count);
		return count;
	}
	
	
	bool Mapper2d::
	AddReference(index_t source_index,
							 ssize_t target_ix, ssize_t target_iy,
							 int value)
	{
		// We need to check for grid bounds because we might be called
		// with indices that lay outside the current grid.

		if ( ! m_refmap.valid(target_ix, target_iy)) {
			PVDEBUG("%lu %lu -> %lu %lu target outside refmap grid\n",
							source_index.v0, source_index.v1, target_ix, target_iy);
			return false;							// target outside refmap grid
		}
		
		ssize_t const source_ix(source_index.v0);
		ssize_t const source_iy(source_index.v1);
		if ( ! m_linkmap.valid(source_ix, source_iy)) {
			PVDEBUG("%lu %lu -> %lu %lu source outside linkmap grid\n",
							source_ix, source_iy, target_ix, target_iy);
			return false;							// source outside linkmap grid
		}
		
		ref_s & ref(m_refmap.at(target_ix, target_iy));
		if (ref.forward.find(source_index) != ref.forward.end()) {
			PVDEBUG("%lu %lu -> %lu %lu already referenced\n",
							source_ix, source_iy, target_ix, target_iy);
			return false;							// already in refmap
		}
		
		link_t & link(m_linkmap.at(source_ix, source_iy));
		link.insert(index_t(target_ix, target_iy));
		ref.forward.insert(make_pair(source_index, value));
		ref.reverse.insert(make_pair(value, source_index));
		return true;
	}
	
	
	size_t Mapper2d::
	RemoveBufferedObstacle(index_t source_index,
												 draw_callback * cb)
	{
		PDEBUG("source index %lu %lu\n", source_index.v0, source_index.v1);
		
		// We need to check for grid bounds because we might be called
		// with indices that lay outside the current grid.
		
		ssize_t const six(source_index.v0);
		ssize_t const siy(source_index.v1);
		if ( ! m_travmap->IsValid(six, siy))
			return 0;
		
		ref_s & ref(m_refmap.at(six, siy));
		if(ref.reverse.empty()){
			m_travmap->SetValue(six, siy, freespace, cb);
			PDEBUG("set source to freespace\n");
		}
		else{
			m_travmap->SetValue(six, siy, ref.reverse.rbegin()->first, cb);
			PDEBUG("set source to %d\n", ref.reverse.rbegin()->first);
		}
		
		size_t count(1);
		link_t & link(m_linkmap.at(six, siy));
		for(link_t::iterator il(link.begin()); il != link.end(); ++il){
			const index_t target_index(*il);
			ssize_t const tix(target_index.v0);
			ssize_t const tiy(target_index.v1);
			int influence, new_target_value;
			if(RemoveReference(source_index, target_index,
												 influence, new_target_value)){
				int old_target_value;
				if( ! m_travmap->GetValue(tix, tiy, old_target_value)){
					PDEBUG_OUT("BUG (harmless): target %zd %zd not in travmap.\n",
										 tix, tiy);
					continue;
				}
				if(old_target_value != new_target_value){
					m_travmap->SetValue(tix, tiy, new_target_value, cb);
					++count;
				}
			}
		}
		link.clear();
		
		return count;
	}
	
	
	bool Mapper2d::
	RemoveReference(index_t source_index,
									index_t target_index,
									int & influence, int & new_value)
	{
		ref_s & ref(m_refmap.at(target_index.v0, target_index.v1));
		const fwd_t::iterator fwd(ref.forward.find(source_index));
		if(fwd == ref.forward.end()){
			PDEBUG("source %lu %lu not in refmap\n",
							source_index.v0, source_index.v1);
			return false;						// source not in refmap (bug in caller?)
		}
		influence = fwd->second;
		
		// remove link from m_refmap
		ref.forward.erase(fwd);
		for(rev_t::iterator bwd(ref.reverse.find(influence));
				bwd != ref.reverse.end(); ++bwd){
			if(bwd->second == source_index){
				ref.reverse.erase(bwd);
				if(ref.reverse.empty()){
					PDEBUG("target %ul %ul set to freespace\n",
								 target_index.v0, target_index.v1);
					new_value = freespace;
				}
				else{
					PDEBUG("target %ul %ul set to %d\n",
								 target_index.v0, target_index.v1,
								 ref.reverse.rbegin()->first);
					new_value = ref.reverse.rbegin()->first;
				}
				return true;
			}
			if(bwd->first != influence)
				break;
		}
		PDEBUG_OUT("BUG: %zd %zd -> %zd %zd in fwd but not bwd refmap\n",
							 source_index.v0, source_index.v1,
							 target_index.v0, target_index.v1);
		new_value = freespace; // just to avoid uninitialized memory usage
		return true;
	}
	
	
	size_t Mapper2d::
	SwipedUpdate(const Frame & pose,
							 const Multiscanner::raw_scan_collection_t & scans,
							 draw_callback * cb)
	{
		RWlock::wrsentry sentry(m_trav_rwlock);
		
		// compute sets of swiped and obstacle cells WITHOUT changing travmap etc
		m_freespace_buffer.clear();
		m_obstacle_buffer.clear();
		swipe_cb swipe(m_freespace_buffer, *m_travmap, ws_obstacle);
		for(size_t is(0); is < scans.size(); ++is){			
			const Scan::array_t & scan_data(scans[is]->data);
			const index_t i0(gridframe.GlobalIndex(scans[is]->pose.X(),
																						 scans[is]->pose.Y()));
			for(size_t ir(0); ir < scan_data.size(); ++ir){
				const index_t i1(gridframe.GlobalIndex(scan_data[ir].globx,
																							 scan_data[ir].globy));
				// Allow drawing outside the current grid range, this will
				// potentially lead to growing the travmap once we call
				// AddBufferedObstacle() further below.
				gridframe.DrawDDALine(i0.v0, i0.v1, i1.v0, i1.v1,
															numeric_limits<ssize_t>::min(),
															numeric_limits<ssize_t>::max(),
															numeric_limits<ssize_t>::min(),
															numeric_limits<ssize_t>::max(),
															swipe);
				if(scan_data[ir].in_range){
					int val;
					if(m_travmap->GetValue(i1.v0, i1.v1, val))
						m_obstacle_buffer.insert(i1);
				}
			}
			// remove new obstacles from swiped set (due to grid effects, it
			// would otherwise be possible for a later ray to erase an
			// obstacle seen by an earlier ray)
			for(link_t::const_iterator io(m_obstacle_buffer.begin());
					io != m_obstacle_buffer.end(); ++io)
				m_freespace_buffer.erase(*io);
		}
		
		// update travmap:
		// 1. remove any obstacle that were traversed by rays
		// 2. add  new obstacles, which	potentially grows the travmap
		size_t count(0);
		for(link_t::const_iterator is(m_freespace_buffer.begin());
				is != m_freespace_buffer.end(); ++is)
			count += RemoveBufferedObstacle(*is, cb);
		for(link_t::const_iterator il(m_obstacle_buffer.begin());
				il != m_obstacle_buffer.end(); ++il)
			count += AddBufferedObstacle(*il, cb);
		
		return count;
	}
	
	
	shared_ptr<RDTravmap> Mapper2d::
	CreateRDTravmap() const
	{
		shared_ptr<RDTravmap> rdt(new RDTravmap(m_travmap, m_trav_rwlock));
		return rdt;
	}
	
	
	shared_ptr<WRTravmap> Mapper2d::
	CreateWRTravmap()
	{
		shared_ptr<WRTravmap> wrt(new WRTravmap(m_travmap, m_trav_rwlock));
		return wrt;
	}
	
	
	Mapper2d::buffered_obstacle_adder::
	buffered_obstacle_adder(Mapper2d * _m2d, Mapper2d::draw_callback * _cb)
		: m2d(_m2d),
			cb(_cb),
			count(0)
	{
	}
	
	
	void Mapper2d::buffered_obstacle_adder::
	operator () (ssize_t ix, ssize_t iy)
	{
		count += m2d->AddBufferedObstacle(index_t(ix, iy), cb);
	}
	
	
	size_t Mapper2d::
	AddObstacleCircle(double globx, double globy, double radius,
										draw_callback * cb)
	{
		RWlock::wrsentry sentry(m_trav_rwlock);
		buffered_obstacle_adder boa(this, cb);
		gridframe.DrawGlobalCircle(globx, globy, radius, boa);
		return boa.count;
	}
	
	
	bool Mapper2d::always_grow::
	operator () (TraversabilityMap & travmap,
							 ssize_t ix, ssize_t iy)
	{
		return travmap.Autogrow(ix, iy, travmap.freespace);
	}
	
}
