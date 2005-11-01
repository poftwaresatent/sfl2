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


#ifndef SUNFLOWER_NF1WAVE_H
#define SUNFLOWER_NF1WAVE_H


#include <sfl/gplan/GridLayer.hpp>


namespace sfl {
  
  
  class NF1Wave
  {
  public:
    typedef GridLayer::index_t index_t;
    
    NF1Wave();
    ~NF1Wave();
    
    void Reset();
    void AddSeed(index_t index);
    void Propagate(GridLayer & grid) const;
    index_t SmallestNeighbor(GridLayer & grid, index_t index) const;
    
  private:
    typedef std::vector<index_t> indexlist_t;
    
    static indexlist_t propagation_neighbor;
    static indexlist_t gradient_neighbor;
    indexlist_t m_seed;
  };
  
}

#endif // SUNFLOWER_NF1WAVE_H
