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
#include <iostream>
#include <sstream>
#include <vector>
#include <math.h>

using namespace boost;
using namespace std;


namespace sfl {
  
  
  TraversabilityMap::
  TraversabilityMap()
    : gframe(0, 0, 0, 1), freespace(0), obstacle(127), name("world")
  {
  }
	
	
	TraversabilityMap::
	TraversabilityMap(const GridFrame & origin, size_t ncells_x, size_t ncells_y)
		: gframe(origin), freespace(0), obstacle(127), name("world")
	{
		data.reset(new array2d<int>(ncells_x, ncells_y, freespace)); 
	}
	
	
	TraversabilityMap::
	TraversabilityMap(const GridFrame & origin, size_t ncells_x, size_t ncells_y,
										int _freespace, int _obstacle, const string & _name)
		: gframe(origin), freespace(_freespace), obstacle(_obstacle), name(_name)
	{
		data.reset(new array2d<int>(ncells_x, ncells_y, freespace));
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
		
    vector<vector<int> > data;
    size_t grid_xsize(0);
    
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
				if(line.size() > grid_xsize)
					grid_xsize = line.size();
      }
    }
    size_t grid_ysize(data.size());
    
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
		result->data.reset(new array2d<int>(grid_xsize, grid_ysize,
																				default_traversability));
    result->gframe.Configure(ox, oy, otheta, resolution);
    for(size_t iy(0); iy < grid_ysize; ++iy)
			if(iy < data.size()){
				vector<int> & line(data[iy]);
				for(size_t ix(0); ix < grid_xsize; ++ix)
					if(ix < line.size())
						(*result->data)[ix][grid_ysize - iy - 1] = line[ix];
			}
    
		return result;
  }
  

  bool TraversabilityMap::
  GetValue(double global_x, double global_y, int & value) const
  {
    if( ! data)
      return false;
    const GridFrame::index_t idx(gframe.GlobalIndex(global_x, global_y));
    if( ! data->ValidIndex(idx))
      return false;
    value = (*data)[idx];
    return true;
  }
  

  bool TraversabilityMap::
  GetValue(size_t index_x, size_t index_y, int & value) const
  {
    if( ! data)
      return false;
    if( ! data->ValidIndex(index_x, index_y))
      return false;
    value = (*data)[index_x][index_y];
    return true;
  }
	
	
	bool TraversabilityMap::
	SetValue(double global_x, double global_y, int value,
					 draw_callback * cb)
  {
    if( ! data)
      return false;
    const GridFrame::index_t idx(gframe.GlobalIndex(global_x, global_y));
    if( ! data->ValidIndex(idx))
      return false;
		if(cb)
			(*cb)(idx.v0, idx.v1, (*data)[idx], value);
    (*data)[idx] = value;
    return true;
	}
	
	
	bool TraversabilityMap::
	SetValue(size_t index_x, size_t index_y, int value,
					 draw_callback * cb)
  {
    if( ! data)
      return false;
    if( ! data->ValidIndex(index_x, index_y))
      return false;
		if(cb)
			(*cb)(index_x, index_y, (*data)[index_x][index_y], value);
    (*data)[index_x][index_y] = value;
    return true;
	}
	
	
	void TraversabilityMap::
	DumpMap(ostream * os)
	{
		*os << "# resolution " << gframe.Delta() << "\n"
				<< "# origin " << gframe.X() << " " << gframe.Y()
				<< " " << gframe.Theta() << "\n";
		if(data)
			*os << "# dimension " << data->xsize << " " << data->ysize << "\n";
		*os << "# obstacle " << obstacle << "\n"
				<< "# freespace " << freespace << "\n"
				<< "# default " << 0 << "\n"
				<< "# name " << name << "\n";
		
		if(data)
			for (size_t j(0); j < data->ysize; j++){
				for (size_t i(0); i < data->xsize; i++) 
					*os << (*data) [i][data->ysize - j - 1] << " ";
				*os << "\n";
			}
	}
	

	bool TraversabilityMap::
	SetObst(double global_x, double global_y, draw_callback * cb)
	{
		return SetValue(global_x, global_y, obstacle, cb);
	}
	
	
	bool TraversabilityMap::
	SetObst(size_t index_x, size_t index_y, draw_callback * cb)
	{
		return SetValue(index_x, index_y, obstacle, cb);
	}
	
	
	bool TraversabilityMap::
	SetFree(double global_x, double global_y, draw_callback * cb)
	{
		return SetValue(global_x, global_y, freespace, cb);
	}
	
	
	bool TraversabilityMap::
	SetFree(size_t index_x, size_t index_y, draw_callback * cb)
	{
		return SetValue(index_x, index_y, freespace, cb);
	}
	
	
	bool TraversabilityMap::
	IsObst(double global_x, double global_y) const
	{
		if( ! data)
      return false;
    const GridFrame::index_t idx(gframe.GlobalIndex(global_x, global_y));
    if( ! data->ValidIndex(idx))
      return false;
    return (*data)[idx] >= obstacle;
	}
	
	
	bool TraversabilityMap::
	IsObst(size_t index_x, size_t index_y) const
	{
		if( ! data)
      return false;
    if( ! data->ValidIndex(index_x, index_y))
      return false;
    return (*data)[index_x][index_y] >= obstacle;
	}
	
	
	bool TraversabilityMap::
	IsFree(double global_x, double global_y) const
	{
		if( ! data)
      return false;
    const GridFrame::index_t idx(gframe.GlobalIndex(global_x, global_y));
    if( ! data->ValidIndex(idx))
      return false;
    return (*data)[idx] <= freespace;
	}
	
	
	bool TraversabilityMap::
	IsFree(size_t index_x, size_t index_y) const
	{
		if( ! data)
      return false;
    if( ! data->ValidIndex(index_x, index_y))
      return false;
    return (*data)[index_x][index_y] <= freespace;
	}
	
}
