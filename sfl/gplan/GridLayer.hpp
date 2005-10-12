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



  /**
     \todo OVERKILL
  */
  class GridLayer
  {
  public:
    typedef std::pair<int, int> index_t;


    GridLayer();
    virtual ~GridLayer();

    virtual void Configure(index_t dimension,
			   double init_value = 0);

    virtual void Set(int i, int j, double value);
    virtual void Set(index_t index, double value);

    virtual void Fill(double value);

    virtual double Get(int i, int j) const;
    virtual double Get(index_t index) const;

    index_t Dimension() const;
    void Dimension(int & width, int & height) const;

    virtual bool Inside(int i, int j) const;
    virtual bool Inside(index_t index) const;



  protected:
    typedef std::vector<double> line_t;
    typedef std::vector<line_t> grid_t;

    index_t _dimension;
    grid_t _grid;
  };



  class NullGridLayer:
    public GridLayer
  {
  public:
    NullGridLayer(): GridLayer() {};
    void Configure(index_t, double) {};
    void Set(int i, int j, double value) {};
    void Set(index_t index, double value) {};
    void Fill(double value) {};
    double Get(int i, int j) const { return 0; };
    double Get(index_t index) const { return 0; };
    bool Inside(int i, int j) const { return true; };
    bool Inside(index_t index) const { return true; };
  };



}

#endif // SUNFLOWER_GRIDLAYER_HPP
