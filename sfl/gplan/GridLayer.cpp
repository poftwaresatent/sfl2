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


#include <sfl/oa/gplan/GridLayer.hpp>
using namespace std;



namespace sfl {



  GridLayer::
  GridLayer()
  {
    Configure(index_t(1, 1));
  }



  GridLayer::
  ~GridLayer()
  {
  }



  void GridLayer::
  Configure(index_t dimension,
	    double init_value)
  {
    if(_dimension == dimension)
      Fill(init_value);
    else{
      _dimension = dimension;
      _grid = grid_t(_dimension.first,
		     line_t(_dimension.second,
			    init_value));
    }
  }



  void GridLayer::
  Set(int i,
      int j,
      double value)
  {
    _grid[i][j] = value;
  }



  void GridLayer::
  Set(index_t index,
      double value)
  {
    _grid[index.first][index.second] = value;
  }



  double GridLayer::
  Get(int i,
      int j)
    const
  {
    return _grid[i][j];
  }



  double GridLayer::
  Get(index_t index)
    const
  {
    return _grid[index.first][index.second];
  }



  void GridLayer::
  Fill(double value)
  {
    fill(_grid.begin(),
	 _grid.end(),
	 line_t(_dimension.second, value));
  }



  void GridLayer::
  Dimension(int & width,
	    int & height)
    const
  {
    width  = _dimension.first;
    height = _dimension.second;
  }



  GridLayer::index_t GridLayer::
  Dimension()
    const
  {
    return _dimension;
  }



  bool GridLayer::
  Inside(int i,
	 int j)
    const
  {
    return
      (i >= 0) && (i < _dimension.first) &&
      (j >= 0) && (j < _dimension.second);
  }



  bool GridLayer::
  Inside(index_t index)
    const
  {
    return
      (index.first  >= 0) && (index.first  < _dimension.first) &&
      (index.second >= 0) && (index.second < _dimension.second);
  }


}
