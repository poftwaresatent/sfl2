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


#include "NF1Wave.hpp"
#include "NF1.hpp"
#include <limits>


namespace sfl {
  
  
  NF1Wave::indexlist_t NF1Wave::propagation_neighbor;
  NF1Wave::indexlist_t NF1Wave::gradient_neighbor;
  
  
  NF1Wave::
  NF1Wave()
  {
    if(propagation_neighbor.size() == 0){
      propagation_neighbor.push_back(index_t( 1,  0));
      propagation_neighbor.push_back(index_t( 0,  1));
      propagation_neighbor.push_back(index_t(-1,  0));
      propagation_neighbor.push_back(index_t( 0, -1));
      
      gradient_neighbor.push_back(index_t( 1,  0));
      gradient_neighbor.push_back(index_t( 1,  1));
      gradient_neighbor.push_back(index_t( 0,  1));
      gradient_neighbor.push_back(index_t(-1,  1));
      gradient_neighbor.push_back(index_t(-1,  0));
      gradient_neighbor.push_back(index_t(-1, -1));
      gradient_neighbor.push_back(index_t( 0, -1));
      gradient_neighbor.push_back(index_t( 1, -1));
    }
  }
  
  
  void NF1Wave::
  Propagate(GridLayer & grid)
    const
  {
    indexlist_t current;
    indexlist_t next = m_seed;
    
    while(next.size() != 0){
      next.swap(current);
      next.clear();
      
      for(indexlist_t::const_iterator i_cell(current.begin());
	  i_cell != current.end();
	  ++i_cell){
	double nextval(grid.Get(*i_cell) + 1);
	
	for(indexlist_t::const_iterator i_nbor(propagation_neighbor.begin());
	    i_nbor != propagation_neighbor.end();
	    ++i_nbor){
	  index_t nbor(*i_nbor);
	  nbor.first  += i_cell->first;
	  nbor.second += i_cell->second;
	  
	  if( ! grid.Inside(nbor))
	    continue;
	  
	  if(grid.Get(nbor) != NF1::FREE)
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
    double minval(std::numeric_limits<double>::max());
    index_t result(-1, -1);
    
    for(indexlist_t::const_iterator i_nbor(gradient_neighbor.begin());
	i_nbor != gradient_neighbor.end();
	++i_nbor){
      index_t nbor(*i_nbor);
      nbor.first  += index.first;
      nbor.second += index.second;
      
      if( ! grid.Inside(nbor))
	continue;
      
      double gridval = grid.Get(nbor);
      
      if((gridval >= 0)
	 && (gridval < minval)){
	result = nbor;
	minval = gridval;
      }
    }
    
    return result;
  }
  
  
  void NF1Wave::
  Reset()
  {
    m_seed.clear();//erase(m_seed.begin(), m_seed.end());
  }



  void NF1Wave::
  AddSeed(index_t index)
  {
    m_seed.push_back(index);
  }
}
