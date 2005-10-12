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


#include "HoloDrive.hpp"
#include <cmath>


using boost::shared_ptr;
using std::string;


namespace sfl {

  HoloDrive::
  HoloDrive(const string & name,
	    double axislength):
    _axislength(axislength),
    _vx(0),
    _vy(0),
    _omega(0)
  {
  }


  void HoloDrive::
  SetSpeed(double vx,
	   double vy,
	   double omega)
  {
    _vx    = vx;
    _vy    = vy;
    _omega = omega;
  }


  shared_ptr<Frame> HoloDrive::
  NextPose(const Frame & pose,
	   double timestep)
  {
    double dx     = _vx    * timestep;
    double dy     = _vy    * timestep;
    double dtheta = _omega * timestep;

    pose.RotateTo(dx, dy);
    _pose_cache.Set(pose);
    _pose_cache.Add(dx, dy, dtheta);

    return shared_ptr<Frame>(new Frame(_pose_cache));
  }

}
