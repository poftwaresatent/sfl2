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


#ifdef SFL_DEBUG
# define PDEBUG PDEBUG_ERR
#else // ! SFL_DEBUG
# define PDEBUG PDEBUG_OFF
#endif // SFL_DEBUG
#define PVDEBUG PDEBUG_OFF


using namespace std;
using namespace estar;
using namespace boost;


static const double sqrt_of_two(1.41421356237);


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

		virtual void operator () (size_t ix, size_t iy)
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
					 size_t grid_ncells_x,
					 size_t grid_ncells_y,
					 double robot_radius,
					 double _buffer_zone,
					 int _freespace,
					 int _obstacle,
					 const std::string & name,
					 boost::shared_ptr<RWlock> trav_rwlock)
		: xsize(grid_ncells_x),
			ysize(grid_ncells_y),
			freespace(_freespace),
			obstacle(_obstacle),
			ws_obstacle(_obstacle + 1),
			gridframe(_gridframe),
			buffer_zone(_buffer_zone),
			grown_safe_distance(robot_radius + _buffer_zone
														+ _gridframe.Delta() * sqrt_of_two),
			grown_robot_radius(robot_radius + _gridframe.Delta() * sqrt_of_two),
			m_travmap(new TraversabilityMap(_gridframe, grid_ncells_x, grid_ncells_y,
																			freespace, obstacle, name)),
			m_trav_rwlock(trav_rwlock),
			m_sprite(new Sprite(grown_safe_distance, _gridframe.Delta())),
			m_linkmap(grid_ncells_x, grid_ncells_y),
			m_refmap(grid_ncells_x, grid_ncells_y)
	{
	}
	
	
	Mapper2d::
	Mapper2d(double robot_radius,
					 double _buffer_zone,
					 boost::shared_ptr<TraversabilityMap> travmap,
					 boost::shared_ptr<RWlock> trav_rwlock)
		: xsize(travmap->data->xsize),
			ysize(travmap->data->ysize),
			freespace(travmap->freespace),
			obstacle(travmap->obstacle),
			ws_obstacle(travmap->obstacle + 1),
			gridframe(travmap->gframe),
			buffer_zone(_buffer_zone),
			grown_safe_distance(robot_radius + _buffer_zone
														+ gridframe.Delta() * sqrt_of_two),
			grown_robot_radius(robot_radius + gridframe.Delta() * sqrt_of_two),
			m_travmap(travmap),
			m_trav_rwlock(trav_rwlock),
			m_sprite(new Sprite(grown_safe_distance, gridframe.Delta())),
			m_linkmap(xsize, ysize),
			m_refmap(xsize, ysize)
	{
	}
	
	
	boost::shared_ptr<Mapper2d> Mapper2d::
	Create(double robot_radius, double buffer_zone,
				 const std::string & traversability_file,
				 std::ostream * err_os)
	{
		shared_ptr<RWlock> rwl(RWlock::Create("sfl::Mapper2d::m_trav_rwlock"));
		if( ! rwl){
      if(err_os) *err_os << "ERROR in Mapper2d::Create():\n"
												 << "  sfl::RWlock::Create() failed\n";
      return boost::shared_ptr<Mapper2d>();
    }
		
		ifstream trav(traversability_file.c_str());
    if( ! trav){
      if(err_os) *err_os << "ERROR in Mapper2d::Create():\n"
												 << "  invalid traversability file \""
												 << traversability_file << "\".\n";
      return boost::shared_ptr<Mapper2d>();
    }
    shared_ptr<TraversabilityMap>
      traversability(TraversabilityMap::Parse(trav, err_os));
    if( ! traversability){
      if(err_os) *err_os << "ERROR in Mapper2d::Create():\n"
												 << "  TraversabilityMap::Parse() failed on \""
												 << traversability_file << "\".\n";
      return boost::shared_ptr<Mapper2d>();
    }
		if( ! traversability->data){
      if(err_os) *err_os << "ERROR in Mapper2d::Create():\n"
												 << "  no traversability date available in \""
												 << traversability_file << "\".\n";
      return boost::shared_ptr<Mapper2d>();
		}
		
		boost::shared_ptr<Mapper2d>
			result(new Mapper2d(robot_radius, buffer_zone, traversability, rwl));
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
	AddBufferedObstacle(double globx, double globy, draw_callback * cb)
	{
		return AddBufferedObstacle(gridframe.GlobalIndex(globx, globy), cb);
	}
	
	
	size_t Mapper2d::
	AddBufferedObstacle(index_t source_index, draw_callback * cb)
	{
		const ssize_t ix0(source_index.v0);
		const ssize_t iy0(source_index.v1);
		
		PVDEBUG("source index %lu %lu\n", ix0, iy0);
		
		// Check if the cell is in the grid or we already have a "true"
		// (workspace) obstacle here. If not, flag it as the center of a
		// new C-space extension.
		{
			int old_value;
			if( ! m_travmap->GetValue(ix0, iy0, old_value))
				return 0;
			if(ws_obstacle == old_value)
				return 0;
		}
		m_travmap->SetValue(ix0, iy0, ws_obstacle, cb);
		size_t count(1);
		
		PVDEBUG("set source to %d\n", ws_obstacle);
		
		// Perform C-space extension with buffer zone, housekeeping the
		// cell-dependency structure.
    const Sprite::indexlist_t & area(m_sprite->GetArea());
		for(size_t ii(0); ii < area.size(); ++ii){
			
			const ssize_t ix(ix0 + area[ii].x);
			if((ix < 0) || (ix >= xsize))
				continue;								// outside of grid
			
			const ssize_t iy(iy0 + area[ii].y);
			if((iy < 0) || (iy >= ysize))
				continue;								// outside of grid
			
			if((ix0 == ix) && (iy0 == iy))
				continue;								// we've already flagged the center
			
			const double dist(area[ii].r);
			if(dist > grown_safe_distance)
				continue;								// outside buffer: nothing to do
			
			int value(obstacle);
			if(dist > grown_robot_radius){
				const double vv(  freespace * (dist - grown_robot_radius)
												+ obstacle  * (grown_safe_distance - dist));
				value = static_cast<int>(rint(vv / buffer_zone));
			}
			
			PVDEBUG("target %lu %lu to %d\n", ix, iy, value);
			
			if( ! AddReference(source_index, ix, iy, value))
				continue;								// should never happen though...
			
			int old_value;
			if( ! m_travmap->GetValue(ix, iy, old_value))
				continue;								// should never happen though...
			
			if(value > old_value){
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
							 size_t target_ix, size_t target_iy,
							 int value)
	{
		const index_t target_index(target_ix, target_iy);
		ref_s & ref(m_refmap[target_index]);
		if(ref.forward.find(source_index) != ref.forward.end()){
			PVDEBUG("%lu %lu -> %lu %lu already referenced\n",
							source_index.v0, source_index.v1, target_ix, target_iy);
			return false;							// source -> target already in refmap
		}
		link_t & link(m_linkmap[source_index]);
		link.insert(target_index);
		ref.forward.insert(make_pair(source_index, value));
		ref.reverse.insert(make_pair(value, source_index));
		return true;
	}
	
	
	size_t Mapper2d::
	RemoveBufferedObstacle(index_t source_index,
												 draw_callback * cb)
	{
		PDEBUG("source index %lu %lu\n", source_index.v0, source_index.v1);
		
		ref_s & ref(m_refmap[source_index]);
		if(ref.reverse.empty()){
			m_travmap->SetValue(source_index.v0, source_index.v1,
													freespace, cb);
			PDEBUG("set source to freespace\n");
		}
		else{
			m_travmap->SetValue(source_index.v0, source_index.v1,
													ref.reverse.rbegin()->first, cb);
			PDEBUG("set source to %d\n", ref.reverse.rbegin()->first);
		}
		
		size_t count(1);
		link_t & link(m_linkmap[source_index]);
		for(link_t::iterator il(link.begin()); il != link.end(); ++il){
			const index_t target_index(*il);
			int influence, new_target_value;
			if(RemoveReference(source_index, target_index,
												 influence, new_target_value)){
				int old_target_value;
				if( ! m_travmap->GetValue(target_index.v0, target_index.v1,
																	old_target_value)){
					PDEBUG_OUT("BUG (harmless): target %lu %lu not in travmap.\n");
					continue;
				}
				if(old_target_value != new_target_value){
					m_travmap->SetValue(target_index.v0, target_index.v1,
															new_target_value, cb);
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
		ref_s & ref(m_refmap[target_index]);
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
		PDEBUG_OUT("BUG: %lu %lu -> %lu %lu in fwd but not bwd refmap\n",
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
		
		// compute sets of swiped and obstacle cells
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
				gridframe.DrawDDALine(i0.v0, i0.v1, i1.v0, i1.v1,
															xsize, ysize, swipe);
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
		
		// update travmap
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
	
}
