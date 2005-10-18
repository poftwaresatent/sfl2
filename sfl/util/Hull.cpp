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


#include "Hull.hpp"
#include "functors.hpp"


using boost::shared_ptr;
using namespace std;


namespace sfl {


  Hull::
  Hull():
    m_npoints(0)
  {
  }
  
  
  Hull::
  Hull(const Hull & original):
    m_npoints(original.m_npoints)
  {
    for(subhulls_t::const_iterator is(original.m_subhulls.begin());
	is != original.m_subhulls.end();
	++is)
      m_subhulls.push_back(shared_ptr<Polygon>(new Polygon( ** is)));
  }
  
  
  void Hull::
  AddPolygon(const Polygon & polygon)
  {
    m_subhulls.push_back(polygon.CreateConvexHull());
    m_npoints += m_subhulls.back()->GetNPoints();
  }
  
  
  shared_ptr<Hull> Hull::
  CreateGrownHull(double amount)
    const
  {
    shared_ptr<Hull> grown(new Hull());
    for(subhulls_t::const_iterator is(m_subhulls.begin());
	is != m_subhulls.end();
	++is){
      grown->m_subhulls.push_back((*is)->CreateGrownPolygon(amount));
      grown->m_npoints += grown->m_subhulls.back()->GetNPoints();
    }
    return grown;
  }
  
  
  double Hull::
  CalculateRadius()
    const
  {
    double radius(-1);
    for(subhulls_t::const_iterator is(m_subhulls.begin());
	is != m_subhulls.end();
	++is){
      const double r((*is)->CalculateRadius());
      if((radius < 0) || (r > radius))
	radius = r;
    }
    return radius;
  }
  
  
  shared_ptr<Line> Hull::
  GetLine(int index)
    const
  {
    if(index < 0)
      return shared_ptr<Line>();
    
    for(subhulls_t::const_iterator is(m_subhulls.begin());
	is != m_subhulls.end();
	++is){
      const int npoints((*is)->GetNPoints());
      if(index < npoints)
	return (*is)->GetLine(index);
      index -= npoints;
    }
    
    return shared_ptr<Line>();
  }
  
  
  bool Hull::
  Contains(double x,
	   double y)
    const
  {
    for(subhulls_t::const_iterator is(m_subhulls.begin());
	is != m_subhulls.end();
	++is)
      if((*is)->Contains(x, y))
	return true;
    return false;
  }
  
  
  shared_ptr<const Polygon> Hull::
  GetPolygon(int index)
    const
  {
    if((index < 0) || (index >= (int) m_subhulls.size()))
      return shared_ptr<const Polygon>();
    return m_subhulls[index];
  }
  
}
