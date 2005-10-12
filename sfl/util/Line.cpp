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


#include "Line.hpp"
#include <sfl/util/numeric.hpp>


namespace sfl {


  Line::
  Line(const Point & _p0,
       const Point & _p1):
    p0(_p0),
    p1(_p1)
  {
  }


  Line::
  Line(double x0,
       double y0,
       double x1,
       double y1):
    p0(x0, y0),
    p1(x1, y1)
  {
  }


  void Line::
  TransformTo(const Frame & t)
  {
    p0.TransformTo(t);
    p1.TransformTo(t);
  }


  void Line::
  TransformFrom(const Frame & t)
  {
    p0.TransformFrom(t);
    p1.TransformFrom(t);
  }


  void Line::
  CircleIntersect(double cx,
		  double cy,
		  double cr,
		  double &q1x,
		  double &q1y,
		  double &q2x,
		  double &q2y,
		  bool &valid1,
		  bool &valid2)
    const
  {
    double delta_cx(cx - p0.X());
    double delta_cy(cy - p0.Y());
    double delta_px(p1.X() - p0.X());
    double delta_py(p1.Y() - p0.Y());

    double a(delta_px * delta_px +
	     delta_py * delta_py);
    double c(delta_cx * delta_cx +
	     delta_cy * delta_cy -
	     cr * cr);
    double b(- 2 * (delta_cx * delta_px +
		    delta_cy * delta_py));

    double l1, l2;
    switch(QuadraticEquation(a, b, c, l1, l2)){
    case 0:
      valid1 = false;
      valid2 = false;
      break;
    case 1:
      q1x = p0.X() + l1 * delta_px;
      q1y = p0.Y() + l1 * delta_py;
      valid1 = (l1 >= 0) && (l1 <= 1);
      valid2 = false;
      break;
    default:
      q1x = p0.X() + l1 * delta_px;
      q1y = p0.Y() + l1 * delta_py;
      q2x = p0.X() + l2 * delta_px;
      q2y = p0.Y() + l2 * delta_py;
      valid1 = (l1 >= 0) && (l1 <= 1);
      valid2 = (l2 >= 0) && (l2 <= 1);
    }
  }

}
