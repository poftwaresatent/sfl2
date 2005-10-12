/* 
 * Copyright (C) 2005
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


#ifndef SUNFLOWER_HULL_HPP
#define SUNFLOWER_HULL_HPP


#include <sfl/util/Polygon.hpp>
#include <boost/shared_ptr.hpp>


namespace sfl {


  /**
     A hull is represented as a set of convex polygons. It is up to
     the user to define how a given hull is subdivided into convex
     sub-hulls.
  */
  class Hull
  {
  public:
    Hull();
    Hull(const Hull & original);
    
    /**
       Copies the provided polygon, creates its convex hull, and adds
       it to the list of convex sub-hulls.
    */
    void AddPolygon(const Polygon & polygon);
    
    /** Grows the hull by growing each contained sub-hull. */
    boost::shared_ptr<Hull> CreateGrownHull(double amount) const;
    
    /**
       Determines the largest distance from the origin to any point of
       the hull.
    */
    double CalculateRadius() const;
    
    /** \return The total number of sub-hulls. */
    int GetNPolygons() const { return m_subhulls.size(); }
    
    /** \return A pointer to a subhull iff 0<=index<GetNPolygons(), 0
	otherwise. */
    const Polygon * GetPolygon(int index) const;
    
    /** \return The total number of points in the sub-hulls. */
    int GetNPoints() const { return m_npoints; }
    
    /**
       Convenience method for legacy code.
              
       \return A copy of one of the lines of one of the sub-hulls (iff
       0<=index<GetNPoints()).
    */
    boost::shared_ptr<Line> GetLine(int index) const;
    
    /**
       Determine whether a given point lies within any of the
       subhulls.
    */
    bool Contains(double x, double y) const;


  protected:
    typedef std::vector<boost::shared_ptr<Polygon> > subhulls_t;
    
    subhulls_t m_subhulls;
    int m_npoints;
  };
  
}

#endif // SUNFLOWER_HULL_HPP
