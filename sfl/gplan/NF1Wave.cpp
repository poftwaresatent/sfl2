/* 
 * Copyright (C) 2004
 * Swiss Federal Institute of Technology, Lausanne. All rights reserved.
 * 
 * Developed at the Autonomous Systems Lab.
 * Visit our homepage at http://asl.epfl.ch/
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


#include <sfl/oa/gplan/NF1Wave.hpp>
#include <sfl/oa/gplan/GridLayer.hpp>
#include <sfl/oa/gplan/GridPlanner.hpp>



namespace sfl {



  NF1Wave::indexlist_t NF1Wave::__propagation_neighbor;
  NF1Wave::indexlist_t NF1Wave::__gradient_neighbor;



  NF1Wave::
  NF1Wave()
  {
    if(__propagation_neighbor.size() == 0){
      __propagation_neighbor.push_back(index_t( 1,  0));
      __propagation_neighbor.push_back(index_t( 0,  1));
      __propagation_neighbor.push_back(index_t(-1,  0));
      __propagation_neighbor.push_back(index_t( 0, -1));

      __gradient_neighbor.push_back(index_t( 1,  0));
      __gradient_neighbor.push_back(index_t( 1,  1));
      __gradient_neighbor.push_back(index_t( 0,  1));
      __gradient_neighbor.push_back(index_t(-1,  1));
      __gradient_neighbor.push_back(index_t(-1,  0));
      __gradient_neighbor.push_back(index_t(-1, -1));
      __gradient_neighbor.push_back(index_t( 0, -1));
      __gradient_neighbor.push_back(index_t( 1, -1));
    }
  }



  NF1Wave::
  ~NF1Wave()
  {
  }



  void NF1Wave::
  Propagate(GridLayer & grid)
    const
  {
    indexlist_t current;
    indexlist_t next = _seed;
  
    while(next.size() != 0){
      next.swap(current);
      next.clear();
      
      for(indexlist_t::const_iterator i_cell(current.begin());
	  i_cell != current.end();
	  ++i_cell){
	double nextval(grid.Get(*i_cell) + 1);
	
	for(indexlist_t::const_iterator i_nbor(__propagation_neighbor.begin());
	    i_nbor != __propagation_neighbor.end();
	    ++i_nbor){
	  index_t nbor(*i_nbor);
	  nbor.first  += i_cell->first;
	  nbor.second += i_cell->second;
	  
	  if( ! grid.Inside(nbor))
	    continue;
	  
	  if(grid.Get(nbor) != GridPlanner::FREE)
	    continue;
	  
	  grid.Set(nbor, nextval);
	  next.push_back(nbor);
	}
      }
    }
  }



  NF1Wave::index_t NF1Wave::
  SmallestNeighbor(GridLayer & grid,
		   index_t index)
    const
  {
    static const double MAGIC = -123; // hax
    double minval(MAGIC);
    index_t result(-1, -1);

    for(indexlist_t::const_iterator i_nbor(__gradient_neighbor.begin());
	i_nbor != __gradient_neighbor.end();
	++i_nbor){
      index_t nbor(*i_nbor);
      nbor.first  += index.first;
      nbor.second += index.second;
      
      if( ! grid.Inside(nbor))
	continue;
      
      double gridval = grid.Get(nbor);
      
      if((gridval >= 0) &&
	 ((gridval < minval) || (minval == MAGIC))){
	result = nbor;
	minval = gridval;
      }
    }
    
    return result;
  }
  
  
  
}
