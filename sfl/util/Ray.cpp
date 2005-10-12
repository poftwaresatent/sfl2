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


#include "Ray.hpp"
#include <sfl/util/numeric.hpp>
#include <cmath>


namespace sfl {


  Ray::
  Ray(double x,
      double y,
      double theta):
    _frame(x, y, theta)
  {
  }


  Ray::
  Ray(const Frame & frame):
    _frame(frame)
  {
  }


  void Ray::
  SetAngle(double angle)
  {
    _frame.Set(_frame.X(), _frame.Y(), angle);
  }


  double Ray::
  Intersect(const Line & line)
    const
  {
    // describe line as {point, vector} in global frame
    double p0x(line.X0());
    double p0y(line.Y0());
    double v0x(line.X1() - p0x);
    double v0y(line.Y1() - p0y);
   
    // determinant of intersection matrix
    double det(_frame.Costheta() * v0y -
	       v0x               * _frame.Sintheta());
   
    // check if ray is parallel
    if(absval(det) <= 1e-9)		// hax: hardcoded epsilon
      return -1;

    // look where the ray intersects with the line segment
    double mu0((_frame.Costheta() * (_frame.Y() - p0y)  -
		_frame.Sintheta() * (_frame.X() - p0x)) /
	       det);
  
    // check if it intersects inside the segment
    if((mu0 < 0) || (mu0 > 1))
      return -1;

    // calculate distance from ray origin to intersection
    double mu1((v0x * (_frame.Y() - p0y)  -
		v0y * (_frame.X() - p0x)) /
	       det);
  
    // check if it's a forward intersection
    if(mu1 < 0)
      return -1;

    return mu1;
  }

}
