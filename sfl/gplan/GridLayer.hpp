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


#ifndef SUNFLOWER_GRIDLAYER_HPP
#define SUNFLOWER_GRIDLAYER_HPP


#include <vector>
#include <utility>


namespace sfl {



  class GridLayer
  {
  public:
    typedef std::pair<int, int> index_t;
    
    GridLayer() { Configure(index_t(1, 1)); }
    
    void Configure(index_t dimension, double init_value = 0);
    void Fill(double value);
    
    void Set(int i, int j, double value) { m_grid[i][j] = value; }
    void Set(index_t index, double value)
    { m_grid[index.first][index.second] = value; }
    
    double Get(int i, int j) const { return m_grid[i][j]; }
    double Get(index_t index) const
    { return m_grid[index.first][index.second]; }
    
    index_t Dimension() const { return m_dimension; }
    void Dimension(int & width, int & height) const
    { width  = m_dimension.first;
      height = m_dimension.second; }
    
    bool Inside(int i, int j) const;
    bool Inside(index_t index) const;
    
  protected:
    typedef std::vector<double> line_t;
    typedef std::vector<line_t> grid_t;
    
    index_t m_dimension;
    grid_t m_grid;
  };
  
}

#endif // SUNFLOWER_GRIDLAYER_HPP
