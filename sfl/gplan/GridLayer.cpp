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


#include "GridLayer.hpp"


namespace sfl {
  
  
  void GridLayer::
  Configure(index_t dimension,
	    double init_value)
  {
    if(m_dimension == dimension)
      Fill(init_value);
    else{
      m_dimension = dimension;
      m_grid = grid_t(m_dimension.first,
		      line_t(m_dimension.second,
			     init_value));
    }
  }
  
  
  void GridLayer::
  Fill(double value)
  {
    fill(m_grid.begin(),
	 m_grid.end(),
	 line_t(m_dimension.second, value));
  }
  
  
  bool GridLayer::
  Inside(int i,
	 int j)
    const
  {
    return
      (i >= 0) && (i < m_dimension.first) &&
      (j >= 0) && (j < m_dimension.second);
  }
  
  
  bool GridLayer::
  Inside(index_t index)
    const
  {
    return
      (index.first  >= 0) && (index.first  < m_dimension.first) &&
      (index.second >= 0) && (index.second < m_dimension.second);
  }
  
}
