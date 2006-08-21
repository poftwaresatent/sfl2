/* 
 * Copyright (C) 2005 Roland Philippsen <roland dot philippsen at gmx net>
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


#ifndef SUNFLOWER_ARRAY2D_HPP
#define SUNFLOWER_ARRAY2D_HPP


#include <boost/scoped_array.hpp>


namespace sfl {
  
  
  /**
     Simple 2D-array with "self destroying" underlying data.
     
     \note If you want to put this into an STL container, wrap it into a
     boost::shared_ptr to avoid problems with the non-copyable
     boost::scoped_array fields.
  */
  template<typename T>
  class array2d
  {
  public:
    typedef boost::scoped_array<T> inner_t;
    typedef boost::scoped_array<inner_t> outer_t;
    
    array2d(size_t xsize, size_t ysize)
      : data(new inner_t[xsize])
    { for(size_t ix(0); ix < xsize; ++ix) data[ix].reset(new T[ysize]); }
    
    array2d(size_t xsize, size_t ysize, const T & init)
      : data(new inner_t[xsize])
    {
      for(size_t ix(0); ix < xsize; ++ix){
	data[ix].reset(new T[ysize]);
	for(size_t iy(0); iy < ysize; ++iy) data[ix][iy] = init;
      }
    }
    
    inner_t & operator [] (size_t ix)
    { return data[ix]; }
    
    const inner_t & operator [] (size_t ix) const
    { return data[ix]; }
    
    outer_t data;
  };
  
}

#endif // SUNFLOWER_ARRAY2D_HPP
