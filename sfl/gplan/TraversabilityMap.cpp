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


using namespace boost;
using namespace std;


namespace sfl {
  
  
  TraversabilityMap::
  TraversabilityMap()
    : gframe(0, 0, 0, 1), freespace(0), obstacle(127), name("world")
  {
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
  
}
