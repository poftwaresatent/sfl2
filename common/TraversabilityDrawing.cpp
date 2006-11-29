/* 
 * Copyright (C) 2006
 * Swiss Federal Institute of Technology, Zurich. All rights reserved.
 * 
 * Developed at the Autonomous Systems Lab.
 * Visit our homepage at http://www.asl.ethz.ch/
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


#include "TraversabilityDrawing.hpp"
#include "wrap_gl.hpp"
#include <sfl/util/numeric.hpp>
#include <sfl/gplan/TraversabilityMap.hpp>
#include <math.h>


using namespace sfl;
using namespace boost;
using namespace std;


namespace npm {
  
  
  TraversabilityDrawing::
  TraversabilityDrawing(const string & name,
			shared_ptr<const sfl::TraversabilityMap> tm)
    : Drawing(name), m_tm(tm)
  {
  }
  
  
  void TraversabilityDrawing::
  Draw()
  {
    if( ! m_tm)
      return;
    if( ! m_tm->data)
      return;
    
    glMatrixMode(GL_MODELVIEW);
    glPushMatrix();
    glTranslated(m_tm->gframe.X(), m_tm->gframe.Y(), 0);
    glRotated(180 * m_tm->gframe.Theta() / M_PI, 0, 0, 1);
    glScaled(m_tm->gframe.Delta(), m_tm->gframe.Delta(), 1);
    
    glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
    const double cscale(1.0 / (m_tm->maxdata - m_tm->mindata));
    for(size_t ix(0); ix < m_tm->data->xsize; ++ix)
      for(size_t iy(0); iy < m_tm->data->ysize; ++iy){
	const int value((*m_tm->data)[ix][iy]);
	const double blue((value - m_tm->mindata) * cscale);
	double green(blue);
	if(value <= m_tm->freespace)
	  green = minval(1.0, green * 2 + 0.2);
	double red(blue);
	if(value >= m_tm->obstacle)
	  red = minval(1.0, red * 2);
	glColor3d(red, green, blue);
	glRectd(ix - 0.5, iy - 0.5, ix + 0.5, iy + 0.5);
      }
    
    glMatrixMode(GL_MODELVIEW);
    glPopMatrix();
  }

}
