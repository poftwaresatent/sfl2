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


#include "Frame.hpp"
#include "numeric.hpp"
#include <cmath>
#include <iostream>


namespace sfl {

  Frame::
  Frame():
    _x(0),
    _y(0),
    _theta(0),
    _sintheta(0),
    _costheta(1)
  {
  }


  Frame::
  Frame(double x,
	double y,
	double theta):
    _x(x),
    _y(y),
    _theta(mod2pi(theta)),
    _sintheta(sin(theta)),
    _costheta(cos(theta))
  {
  }


  void Frame::
  Set(const Frame & f)
  {
    _x = f._x;
    _y = f._y;
    _theta = f._theta;
    _sintheta = f._sintheta;
    _costheta = f._costheta;
  }


  void Frame::
  Set(double x,
      double y,
      double theta)
  {
    _x = x;
    _y = y;
    _theta = mod2pi(theta);
    _sintheta = sin(theta);
    _costheta = cos(theta);
  }


  void Frame::
  Get(double & x,
      double & y,
      double & theta)
    const
  {
    x = _x;
    y = _y;
    theta = _theta;
  }


  double Frame::
  X()
    const
  {
    return _x;
  }


  double Frame::
  Y()
    const
  {
    return _y;
  }


  double Frame::
  Theta()
    const
  {
    return _theta;
  }


  double Frame::
  Costheta()
    const
  {
    return _costheta;
  }


  double Frame::
  Sintheta()
    const
  {
    return _sintheta;
  }


  void Frame::
  To(double & x,
     double & y)
    const
  {
    double tmpx(x * _costheta - y * _sintheta);
    double tmpy(x * _sintheta + y * _costheta);
    x = tmpx + _x;
    y = tmpy + _y;
  }


  void Frame::
  To(Frame & frame)
    const
  {
    To(frame._x, frame._y);
    RotateTo(frame._costheta, frame._sintheta);
    frame._theta = mod2pi(frame._theta + _theta);
  }


  void Frame::
  To(double & x,
     double & y,
     double & theta)
    const
  {
    To(x, y);
    theta = mod2pi(theta + _theta);
  }


  void Frame::
  RotateTo(double & x,
	   double & y)
    const
  {
    double tmpx(x * _costheta - y * _sintheta);
    double tmpy(x * _sintheta + y * _costheta);
    x = tmpx;
    y = tmpy;
  }


  void Frame::
  RotateTo(Frame & frame)
    const
  {
    RotateTo(frame._x, frame._y);
    RotateTo(frame._costheta, frame._sintheta);
    frame._theta = mod2pi(frame._theta + _theta);
  }


  void Frame::
  From(double & x,
       double & y)
    const
  {
    x -= _x;
    y -= _y;
    double tmpx( x * _costheta + y * _sintheta);
    double tmpy(-x * _sintheta + y * _costheta);
    x = tmpx;
    y = tmpy;
  }


  void Frame::
  From(Frame & frame)
    const
  {
    From(frame._x, frame._y);
    RotateFrom(frame._costheta, frame._sintheta);
    frame._theta = mod2pi(frame._theta - _theta);
  }


  void Frame::
  From(double & x,
       double & y,
       double & theta)
    const
  {
    From(x, y);
    theta = mod2pi(theta - _theta);
  }
  
  
  void Frame::
  RotateFrom(double & x,
	     double & y)
    const
  {
    double tmpx(   x * _costheta + y * _sintheta);
    double tmpy( - x * _sintheta + y * _costheta);
    x = tmpx;
    y = tmpy;
  }


  void Frame::
  RotateFrom(Frame & frame)
    const
  {
    RotateFrom(frame._x, frame._y);
    RotateFrom(frame._costheta, frame._sintheta);
    frame._theta = mod2pi(frame._theta - _theta);
  }


  void Frame::
  Add(double dx,
      double dy,
      double dtheta)
  {
    _x += dx;
    _y += dy;
    _theta = mod2pi(_theta + dtheta);
    _costheta = cos(_theta);
    _sintheta = sin(_theta);
  }
  

  std::ostream & operator<<(std::ostream & os, const Frame & f) {
    os << "(" << f._x <<  ", " <<  f._y <<  ", " <<  f._theta <<  ")";
    return os;
  }
  
}
