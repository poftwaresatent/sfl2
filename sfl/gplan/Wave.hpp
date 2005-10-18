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


#ifndef SUNFLOWER_WAVE_HPP
#define SUNFLOWER_WAVE_HPP



#include <sfl/gplan/GridLayer.hpp>
#include <vector>



namespace sfl {



  class Wave
  {
  public:
    typedef GridLayer::index_t index_t;
  
    Wave();
    virtual ~Wave();
  
    void Reset();
    void AddSeed(index_t index);

    virtual void Propagate(GridLayer & grid) const = 0;



  protected:
    typedef std::vector<index_t> indexlist_t;

    indexlist_t _seed;
  };



}

#endif // SUNFLOWER_WAVE_HPP
