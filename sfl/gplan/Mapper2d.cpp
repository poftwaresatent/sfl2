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


#ifdef DEBUG
# define SFL_DEBUG
#endif // DEBUG

#ifdef SFL_DEBUG
# define PDEBUG PDEBUG_ERR
#else // ! DEBUG
# define PDEBUG PDEBUG_OFF
#endif // DEBUG
#define PVDEBUG PDEBUG_OFF


using namespace std;
using namespace estar;
using namespace boost;


static const double sqrt_of_two(1.41421356237);


namespace sfl {
	
	
	Mapper2d::
	Mapper2d(const GridFrame & gridframe,
					 size_t grid_ncells_x,
					 size_t grid_ncells_y,
					 double robot_radius,
					 double buffer_zone,
					 int freespace,
					 int obstacle,
					 const std::string & name)
		: m_xsize(grid_ncells_x),
			m_ysize(grid_ncells_y),
			m_freespace(freespace),
			m_obstacle(obstacle),
			m_ws_obstacle(obstacle + 1),
			m_gridframe(gridframe),
			m_buffer_zone(buffer_zone),
			m_grown_safe_distance(robot_radius + buffer_zone
														+ gridframe.Delta() * sqrt_of_two),
			m_grown_robot_radius(robot_radius + gridframe.Delta() * sqrt_of_two),
			m_travmap(new TraversabilityMap(gridframe, grid_ncells_x, grid_ncells_y,
																			freespace, obstacle, name)),
			m_sprite(new Sprite(m_grown_safe_distance, gridframe.Delta())),
			m_refmap(grid_ncells_x, grid_ncells_y)
	{
	}
	
	
	Mapper2d::
	Mapper2d(double robot_radius,
					 double buffer_zone,
					 boost::shared_ptr<TraversabilityMap> travmap)
		: m_xsize(travmap->data->xsize),
			m_ysize(travmap->data->ysize),
			m_freespace(travmap->freespace),
			m_obstacle(travmap->obstacle),
			m_ws_obstacle(travmap->obstacle + 1),
			m_gridframe(travmap->gframe),
			m_buffer_zone(buffer_zone),
			m_grown_safe_distance(robot_radius + buffer_zone
														+ m_gridframe.Delta() * sqrt_of_two),
			m_grown_robot_radius(robot_radius + m_gridframe.Delta() * sqrt_of_two),
			m_travmap(travmap),
			m_sprite(new Sprite(m_grown_safe_distance, m_gridframe.Delta())),
			m_refmap(m_xsize, m_ysize)
	{
	}
	
	
	boost::shared_ptr<Mapper2d> Mapper2d::
	Create(double robot_radius, double buffer_zone,
				 const std::string & traversability_file,
				 std::ostream * err_os)
	{
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
		boost::shared_ptr<Mapper2d> result(new Mapper2d(robot_radius,
																										buffer_zone,
																										traversability));
		return result;
	}
	
	
	size_t Mapper2d::
	Update(const Frame & pose, size_t length, double * locx, double * locy,
				 draw_callback * cb)
	{
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
		const GridFrame::index_t
			source_index(m_gridframe.GlobalIndex(globx, globy));
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
			if(m_ws_obstacle == old_value)
				return 0;
		}
		m_travmap->SetValue(ix0, iy0, m_ws_obstacle, cb);
		size_t count(1);
		
		PVDEBUG("set source to %d\n", m_ws_obstacle);
		
		// Perform C-space extension with buffer zone, housekeeping the
		// cell-dependency structure.
    const Sprite::indexlist_t & area(m_sprite->GetArea());
		for(size_t ii(0); ii < area.size(); ++ii){
			
			const ssize_t ix(ix0 + area[ii].x);
			if((ix < 0) || (ix >= m_xsize))
				continue;								// outside of grid
			
			const ssize_t iy(iy0 + area[ii].y);
			if((iy < 0) || (iy >= m_ysize))
				continue;								// outside of grid
			
			if((ix0 == ix) && (iy0 == iy))
				continue;								// we've already flagged the center
			
			const double dist(area[ii].r);
			if(dist > m_grown_safe_distance)
				continue;								// outside buffer: nothing to do
			
			int value(m_obstacle);
			if(dist > m_grown_robot_radius){
				const double vv(  m_freespace * (dist - m_grown_robot_radius)
												+ m_obstacle  * (m_grown_safe_distance - dist));
				value = static_cast<int>(rint(vv / m_buffer_zone));
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
	AddReference(GridFrame::index_t source_index,
							 size_t target_ix, size_t target_iy,
							 int value)
	{
		const GridFrame::index_t target_index(target_ix, target_iy);
		ref_s & ref(m_refmap[target_index]);
		if(ref.forward.find(source_index) != ref.forward.end()){
			PVDEBUG("%lu %lu -> %lu %lu already referenced\n",
							source_index.v0, source_index.v1, target_ix, target_iy);
			return false;							// source -> target already in refmap
		}
		ref.forward.insert(make_pair(source_index, value));
		ref.reverse.insert(make_pair(value, source_index));
		return true;
	}
	
}
