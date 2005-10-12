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


#include "Pose.hpp"


namespace sfl {


  Pose::
  Pose():
    Frame(),
    _sxx(1),
    _syy(1),
    _stt(1),
    _sxy(0),
    _sxt(0),
    _syt(0)
  {
  }


  Pose::
  Pose(const Frame & frame):
    Frame(frame),
    _sxx(1),
    _syy(1),
    _stt(1),
    _sxy(0),
    _sxt(0),
    _syt(0)
  {
  }
  
  
  Pose::
  Pose(double x, double y, double theta):
    Frame(x, y, theta),
    _sxx(1),
    _syy(1),
    _stt(1),
    _sxy(0),
    _sxt(0),
    _syt(0)
  {
  }


  Pose::
  Pose(const Frame & frame,
       double sxx, double syy, double stt,
       double sxy, double sxt, double syt):
    Frame(frame),
    _sxx(sxx),
    _syy(syy),
    _stt(stt),
    _sxy(sxy),
    _sxt(sxt),
    _syt(syt)
  {
  }


  Pose::
  Pose(double x, double y, double theta,
       double sxx, double syy, double stt,
       double sxy, double sxt, double syt):
    Frame(x, y, theta),
    _sxx(sxx),
    _syy(syy),
    _stt(stt),
    _sxy(sxy),
    _sxt(sxt),
    _syt(syt)
  {
  }


  double Pose::
  Sxx()
    const
  {
    return _sxx;
  }


  double Pose::
  Syy()
    const
  {
    return _syy;
  }


  double Pose::
  Stt()
    const
  {
    return _stt;
  }


  double Pose::
  Sxy()
    const
  {
    return _sxy;
  }


  double Pose::
  Sxt()
    const
  {
    return _sxt;
  }


  double Pose::
  Syt()
    const
  {
    return _syt;
  }

  
  void Pose::
  Set(double sxx, double syy, double stt,
      double sxy, double sxt, double syt)
  {
    _sxx = sxx;
    _syy = syy;
    _stt = stt;
    _sxy = sxy;
    _sxt = sxt;
    _syt = syt;
  }
  
}
