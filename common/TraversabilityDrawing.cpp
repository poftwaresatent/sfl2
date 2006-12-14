/* -*- mode: C++; tab-width: 2 -*- */
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
												shared_ptr<TraversabilityProxy> proxy)
    : Drawing(name), m_proxy(proxy)
  {
  }
  
  
  TraversabilityDrawing::
  TraversabilityDrawing(const string & name,
			TraversabilityProxy * proxy)
    : Drawing(name), m_proxy(proxy)
  {
  }
  
  
  void TraversabilityDrawing::
  Draw()
  {
		const TraversabilityMap * tm(m_proxy->Get());
    if( ! tm)
      return;
    if( ! tm->data)
      return;
    
    glMatrixMode(GL_MODELVIEW);
    glPushMatrix();
    glTranslated(tm->gframe.X(), tm->gframe.Y(), 0);
    glRotated(180 * tm->gframe.Theta() / M_PI, 0, 0, 1);
    glScaled(tm->gframe.Delta(), tm->gframe.Delta(), 1);
    
    glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
    const double cscale(1.0 / (tm->maxdata - tm->mindata));
    for(size_t ix(0); ix < tm->data->xsize; ++ix)
      for(size_t iy(0); iy < tm->data->ysize; ++iy){
	const int value((*tm->data)[ix][iy]);
	const double blue((value - tm->mindata) * cscale);
	double green(blue);
	if(value <= tm->freespace)
	  green = minval(1.0, green * 2 + 0.2);
	double red(blue);
	if(value >= tm->obstacle)
	  red = minval(1.0, red * 2);
	glColor3d(red, green, blue);
	glRectd(ix - 0.5, iy - 0.5, ix + 0.5, iy + 0.5);
      }
    glMatrixMode(GL_MODELVIEW);
    glPopMatrix();
  }

}
