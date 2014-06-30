/* Nepumuk Mobile Robot Simulator v2
 *
 * Copyright (C) 2014 Roland Philippsen. All rights reserved.
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

#include "DifferentialDrive.hpp"
#include "Object.hpp"


namespace npm2 {
  
  
  DifferentialDrive::
  DifferentialDrive ()
    : radius_left_ (1.0),
      radius_right_ (1.0),
      wheel_base_ (1.0),
      object_ (0),
      speed_left_ (0.0),
      speed_right_ (0.0)
  {
  }
  
  
  void DifferentialDrive::
  setSpeed (double wl, double wr)
  {
    speed_left_ = wl;
    speed_right_ = wr;
  }
  
  
  void DifferentialDrive::
  integrate (double dt)
  {
    if ( ! object_) {
      return;
    }
    
    double const dl (radius_left_ * speed_left_);
    double const dr (radius_right_ * speed_right_);
    double const vtrans ((dl + dr) / 2.0);
    double const vrot ((dr - dl) / wheel_base_);
    double dx (vtrans * dt);
    double dy (0.0);
    double dth (vrot * dt);
    
    object_->motion_.RotateTo (dx, dy);
    object_->motion_.Add (dx, dy, dth);
  }

}
