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
	TraversabilityMap(const Frame &origin, double resolution, double xSize, double ySize)
		: gframe(origin, resolution)
		, freespace(0), obstacle(127), name("world")
	{
		/*		vector<vector<int> > data;
					vector<int> line;
					int value(0);
					cout << "Allocating Map" << endl;
					for (int j(0); j<(ySize/resolution); j++)
					{
					for (int i(0); i<(xSize/resolution); i++)
					{
					line.push_back(value);
					}
					data.push_back(line);
					cout << "Allocated Line " << j << endl;
					}*/
		data.reset(new array2d<int> (static_cast<size_t>(rint(xSize/resolution)),
																 static_cast<size_t>(rint(ySize/resolution)),
																 freespace)); 
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
    
    vector<vector<int> > data;
    size_t grid_xsize(0);
    int maxdata(numeric_limits<int>::min());
    int mindata(numeric_limits<int>::max());
    
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
      while(tls >> value){
				line.push_back(value);
				if(value > maxdata)
					maxdata = value;
				if(value < mindata)
					mindata = value;
      }
      if( ! line.empty()){
				data.push_back(line);
				if(line.size() > grid_xsize)
					grid_xsize = line.size();
      }
    }
    const size_t grid_ysize(data.size());
    
    if(grid_xsize < 1){
      if(os) *os << "ERROR: grid_xsize == " << grid_xsize << "\n";
      return shared_ptr<TraversabilityMap>();
    }
    if(grid_ysize < 1){
      if(os) *os << "ERROR: grid_ysize == " << grid_ysize << "\n";
      return shared_ptr<TraversabilityMap>();
    }
    
    result->data.reset(new array2d<int>(grid_xsize, grid_ysize,
																				default_traversability));
    for(size_t iy(0); iy < grid_ysize; ++iy){
      vector<int> & line(data[iy]);
      for(size_t ix(0); ix < line.size(); ++ix)
				(*result->data)[ix][grid_ysize - iy - 1] = line[ix];
    }
    result->mindata = mindata;
    result->maxdata = maxdata;
    
    result->gframe.Configure(ox, oy, otheta, resolution);
    
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
	SetValue(double global_x, double global_y, int value,
					 GridFrame::draw_callback * cb)
  {
    if( ! data)
      return false;
    const GridFrame::index_t idx(gframe.GlobalIndex(global_x, global_y));
// 		cout << "Trying to add point at " << global_x << " " << global_y 
// 				 << " to cell " << idx.v0 << " " << idx.v1 << endl;
    if( ! data->ValidIndex(idx))
      return false;
    (*data)[idx] = value;
		if(cb)
			(*cb)(idx.v0, idx.v1);
    return true;
	}

	void TraversabilityMap::
	DumpMap(ostream * os){
		*os << "# resolution " << gframe.Delta() << endl;
		*os << "# origin " << gframe.X() << " " << gframe.Y() << " " << gframe.Theta() << endl;
		*os << "# obstacle " << obstacle << endl;
		*os << "# freespace " << freespace << endl;
		*os << "# default " << 0 << endl;
		*os << "# name " << name << endl;;

		for (size_t j(0); j < data->ysize; j++)
			{
				for (size_t i(0); i < data->xsize; i++) 
					*os << (*data) [i][data->ysize - j - 1] << " ";
				*os << endl;
			}
	}
	
}
