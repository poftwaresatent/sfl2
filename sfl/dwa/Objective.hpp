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


#ifndef SUNFLOWER_OBJECTIVE_HPP
#define SUNFLOWER_OBJECTIVE_HPP


#include <sfl/util/array2d.hpp>
#include <iosfwd>


namespace sfl {
  
  
  class DynamicWindow;
  
  
  class Objective
  {
  public:
    Objective(const DynamicWindow & dynamic_window);

    /** \note empty default implementation */
    virtual ~Objective() {}
    
    /** \note empty default implementation */
    virtual void Initialize(std::ostream * progress_stream) {}
    
    /** \pre all indices must be valid. */
    void Rescale(size_t qdlMin, size_t qdlMax,
		 size_t qdrMin, size_t qdrMax);

    /** \pre all indices must be valid. */
    double Value(size_t qdlIndex, size_t qdrIndex) const
    { return m_value[qdlIndex][qdrIndex]; }
    
    /** \pre all indices must be valid. */
    double Min(size_t qdlMin, size_t qdlMax,
	       size_t qdrMin, size_t qdrMax) const;

    /** \pre all indices must be valid. */
    double Max(size_t qdlMin, size_t qdlMax,
	       size_t qdrMin, size_t qdrMax) const;

    static const double minValue;// = 0;
    static const double maxValue;// = 1;
    const size_t dimension;
    
  protected:
    const DynamicWindow & m_dynamic_window;
    array2d<double> m_value;
  };
  
}

#endif // SUNFLOWER_OBJECTIVE_HPP
