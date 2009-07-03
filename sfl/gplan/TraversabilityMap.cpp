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


#include "TraversabilityMap.hpp"
#include "../util/numeric.hpp"
#include <iostream>
#include <sstream>
#include <vector>
#include <math.h>

using namespace boost;
using namespace std;


static const int default_freespace(0);
static const int default_obstacle(127);


namespace sfl {
  
  
  TraversabilityMap::
  TraversabilityMap()
    : GridRepresentation<int>(),
			freespace(default_freespace),
			obstacle(default_obstacle),
			w_obstacle(default_obstacle+1),
			name("world")
  {
  }
	
	
	TraversabilityMap::
	TraversabilityMap(const GridFrame & origin,
										ssize_t xbegin, ssize_t xend,
										ssize_t ybegin, ssize_t yend)
		: GridRepresentation<int>(origin, xbegin, xend, ybegin, yend, default_freespace),
			freespace(default_freespace),
			obstacle(default_obstacle),
			w_obstacle(default_obstacle+1),
			name("world")
	{
	}
	
	
	TraversabilityMap::
	TraversabilityMap(const GridFrame & origin,
										ssize_t xbegin, ssize_t xend,
										ssize_t ybegin, ssize_t yend,
										int _freespace, int _obstacle, const string & _name)
		: GridRepresentation<int>(origin, xbegin, xend, ybegin, yend, _freespace),
			freespace(_freespace),
			obstacle(_obstacle),
			w_obstacle(_obstacle + 1),
			name(_name)
	{
		grid.resize(xbegin, xend, ybegin, yend, freespace); 
	}
  
	
  shared_ptr<TraversabilityMap> TraversabilityMap::
  Parse(istream & is, ostream * os)
  {
    shared_ptr<TraversabilityMap> result(new TraversabilityMap());
    double ox(result->gframe.X());
    double oy(result->gframe.Y());
    double otheta(result->gframe.Theta());
    double resolution(result->gframe.Delta());
    int default_traversability(0);
    int force_dimx(-1);
    int force_dimy(-1);
		int offx(0);
		int offy(0);
		
    vector<vector<int> > data;
    ssize_t grid_xsize(0);
    
    string textline;
    while(getline(is, textline)){
      istringstream tls(textline);
      if(textline[0] == '#'){
				tls.ignore(1, '\n');
				string token;
				if(tls >> token){
					if(token == "resolution"){
						tls >> resolution;
						if( ! tls){
							if(os) *os << "ERROR: could not parse resolution from \""
												 << tls.str() << "\"\n";
							return shared_ptr<TraversabilityMap>();
						}
					}
					else if(token == "origin"){
						tls >> ox >> oy >> otheta;
						if( ! tls){
							if(os) *os << "ERROR: could not parse origin from \""
												 << tls.str() << "\"\n";
							return shared_ptr<TraversabilityMap>();
						}
					}
					else if(token == "dimension"){
						tls >> force_dimx >> force_dimy;
						if( ! tls){
							if(os) *os << "ERROR: could not parse dimension from \""
												 << tls.str() << "\"\n";
							return shared_ptr<TraversabilityMap>();
						}
						if((force_dimx < 1) || (force_dimy < 1)){
							if(os) *os << "ERROR: invalid grid dimension \"" << tls.str()
												 << "\" (two positive integers expected)\n";
							return shared_ptr<TraversabilityMap>();
						}
					}
					else if(token == "offset"){
						tls >> offx >> offy;
						if( ! tls){
							if(os) *os << "ERROR: could not parse offset from \""
												 << tls.str() << "\"\n";
							return shared_ptr<TraversabilityMap>();
						}
					}
					else if(token == "obstacle"){
						tls >> result->obstacle;
						if( ! tls){
							if(os) *os << "ERROR: could not parse obstacle from \""
												 << tls.str() << "\"\n";
							return shared_ptr<TraversabilityMap>();
						}
					}
					else if(token == "freespace"){
						tls >> result->freespace;
						if( ! tls){
							if(os) *os << "ERROR: could not parse freespace from \""
												 << tls.str() << "\"\n";
							return shared_ptr<TraversabilityMap>();
						}
					}
					else if(token == "default"){
						tls >> default_traversability;
						if( ! tls){
							if(os) *os << "ERROR: could not parse default from \""
												 << tls.str() << "\"\n";
							return shared_ptr<TraversabilityMap>();
						}
					}
					else if(token == "name"){
						tls >> result->name;
						if( ! tls){
							if(os) *os << "ERROR: could not parse name from \""
												 << tls.str() << "\"\n";
							return shared_ptr<TraversabilityMap>();
						}
						if(result->name.empty()){
							if(os) *os << "ERROR: empty names not allowed (from \""
												 << tls.str() << "\")\n";
							return shared_ptr<TraversabilityMap>();
						}
					}
				}
				continue;
      }
			
      vector<int> line;
      int value;
      while(tls >> value)
				line.push_back(value);
      if( ! line.empty()){
				data.push_back(line);
				if(static_cast<ssize_t>(line.size()) > grid_xsize)
					grid_xsize = line.size();
      }
    }
    ssize_t grid_ysize(data.size());
    
    if((force_dimx < 0) && (grid_xsize < 1)){
      if(os) *os << "ERROR: xsize is " << grid_xsize
								 << " and dimension not specified\n";
      return shared_ptr<TraversabilityMap>();
    }
    if((force_dimy < 0) && (grid_ysize < 1)){
      if(os) *os << "ERROR: ysize is " << grid_ysize
								 << " and dimension not specified\n";
      return shared_ptr<TraversabilityMap>();
    }
    
		if(force_dimx > 0)
			grid_xsize = force_dimx;
		if(force_dimy > 0)
			grid_ysize = force_dimy;
		ssize_t const xbegin(offx);
		ssize_t const xend(grid_xsize + offx);
		ssize_t const ybegin(offy);
		ssize_t const yend(grid_ysize + offy);
		
		result->grid.resize(xbegin, xend, ybegin, yend, default_traversability);
    result->gframe.Configure(ox, oy, otheta, resolution);
    for(ssize_t iy(0); iy < grid_ysize; ++iy)
			if(iy < static_cast<ssize_t>(data.size())){
				vector<int> & line(data[iy]);
				for(ssize_t ix(0); ix < grid_xsize; ++ix)
					if(ix < static_cast<ssize_t>(line.size()))
						result->grid.at(ix + offx, grid_ysize - iy - offy - 1) = line[ix];
			}
    
		return result;
  }
  

  bool TraversabilityMap::
  GetValue(double global_x, double global_y, int & value) const
  {
		return GetValueCoord(global_x, global_y, value);
  }
  

  bool TraversabilityMap::
  GetValue(ssize_t index_x, ssize_t index_y, int & value) const
  {
		return GetValueIdx(index_x, index_y, value);
  }
	
	
	bool TraversabilityMap::
	SetValue(double global_x, double global_y, int value,
					 draw_callback * cb)
  {
		const GridFrame::index_t idx(gframe.GlobalIndex(global_x, global_y));
		int old_value;
		if ( ! GetValueIdx(idx.v0, idx.v1, old_value))
			return false;
		if(cb)
			(*cb)(idx.v0, idx.v1, old_value, value);
    grid.at(idx.v0, idx.v1) = value;
    return true;
	}
	
	
	bool TraversabilityMap::
	SetValue(ssize_t index_x, ssize_t index_y, int value,
					 draw_callback * cb)
  {
		int old_value;
		if ( ! GetValueIdx(index_x, index_y, old_value))
			return false;
		if(cb)
			(*cb)(index_x, index_y, grid.at(index_x, index_y), value);
    grid.at(index_x, index_y) = value;
    return true;
	}
	
	
	void TraversabilityMap::
	DumpMap(ostream * os) const
	{
		ssize_t const xsize(grid.xend() - grid.xbegin());
		ssize_t const ysize(grid.yend() - grid.ybegin());
		*os << "# resolution " << gframe.Delta() << "\n"
				<< "# origin " << gframe.X() << " " << gframe.Y()
				<< " " << gframe.Theta() << "\n"
				<< "# w_obstacle " << w_obstacle << "\n"
				<< "# obstacle " << obstacle << "\n"
				<< "# freespace " << freespace << "\n"
				<< "# default " << 0 << "\n"
				<< "# name " << name << "\n"
				<< "# dimension " << xsize << " " << ysize << "\n"
				<< "# offset " << grid.xbegin() << " " << grid.ybegin() << "\n";
		
		for (ssize_t jj(grid.yend() - 1); jj >= grid.ybegin(); --jj) {
			for (ssize_t ii(grid.xbegin()); ii < grid.xend(); ++ii) {
				// should probably use manipulators...
				if (10 > grid.at(ii, jj))
					*os << "  ";
				else if (100 > grid.at(ii, jj))
					*os << " ";
				*os << " " << grid.at(ii, jj);
			}
			*os << "\n";
		}
	}
	
	
	bool TraversabilityMap::
	SetObst(double global_x, double global_y, draw_callback * cb)
	{
		return SetValue(global_x, global_y, obstacle, cb);
	}
	
	
	bool TraversabilityMap::
	SetObst(ssize_t index_x, ssize_t index_y, draw_callback * cb)
	{
		return SetValue(index_x, index_y, obstacle, cb);
	}
	
	
	bool TraversabilityMap::
	SetWObst(double global_x, double global_y, draw_callback * cb)
	{
		return SetValue(global_x, global_y, w_obstacle, cb);
	}
	
	
	bool TraversabilityMap::
	SetWObst(ssize_t index_x, ssize_t index_y, draw_callback * cb)
	{
		return SetValue(index_x, index_y, w_obstacle, cb);
	}
	
	
	bool TraversabilityMap::
	SetFree(double global_x, double global_y, draw_callback * cb)
	{
		return SetValue(global_x, global_y, freespace, cb);
	}
	
	
	bool TraversabilityMap::
	SetFree(ssize_t index_x, ssize_t index_y, draw_callback * cb)
	{
		return SetValue(index_x, index_y, freespace, cb);
	}
	
	
	bool TraversabilityMap::
	IsObst(double global_x, double global_y) const
	{
    const GridFrame::index_t idx(gframe.GlobalIndex(global_x, global_y));
    if( ! grid.valid(idx.v0, idx.v1))
      return false;
    return grid.at(idx.v0, idx.v1) >= obstacle;
	}
	
	
	bool TraversabilityMap::
	IsObst(ssize_t index_x, ssize_t index_y) const
	{
    if( ! grid.valid(index_x, index_y))
      return false;
    return grid.at(index_x, index_y) >= obstacle;
	}
	
	
	bool TraversabilityMap::
	IsWObst(double global_x, double global_y) const
	{
    const GridFrame::index_t idx(gframe.GlobalIndex(global_x, global_y));
    if( ! grid.valid(idx.v0, idx.v1))
      return false;
    return grid.at(idx.v0, idx.v1) >= w_obstacle;
	}
	
	
	bool TraversabilityMap::
	IsWObst(ssize_t index_x, ssize_t index_y) const
	{
    if( ! grid.valid(index_x, index_y))
      return false;
    return grid.at(index_x, index_y) >= w_obstacle;
	}
	
	
	bool TraversabilityMap::
	IsFree(double global_x, double global_y) const
	{
    const GridFrame::index_t idx(gframe.GlobalIndex(global_x, global_y));
    if( ! grid.valid(idx.v0, idx.v1))
      return false;
    return grid.at(idx.v0, idx.v1) <= freespace;
	}
	
	
	bool TraversabilityMap::
	IsFree(ssize_t index_x, ssize_t index_y) const
	{
    if( ! grid.valid(index_x, index_y))
      return false;
    return grid.at(index_x, index_y) <= freespace;
	}
	
	
	bool TraversabilityMap::
	IsValid(ssize_t index_x, ssize_t index_y) const
	{
		return IsValidIdx(index_x, index_y);
	}
	
	
	bool TraversabilityMap::
	IsValid(double global_x, double global_y) const
	{
		return IsValidCoord(global_x, global_y);
	}
	
	
	bool TraversabilityMap::
	Autogrow(ssize_t index_x, ssize_t index_y, int fill_value)
	{
		return AutogrowIdx(index_x, index_y, fill_value);
	}
	
	
	bool TraversabilityMap::
	Autogrow(double global_x, double global_y, int fill_value)
	{
		return AutogrowCoord(global_x, global_y, fill_value);
	}
	
}
