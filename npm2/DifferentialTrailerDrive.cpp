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

#include "DifferentialTrailerDrive.hpp"
#include "Object.hpp"
#include <cmath>


namespace npm2 {
  
  
  DifferentialTrailerDrive::
  DifferentialTrailerDrive ()
    : radius_left_ (1.0),
      radius_right_ (1.0),
      wheel_base_ (1.0),
      hitch_offset_ (1.0),
      trailer_arm_ (1.0),
      tractor_ (0),
      trailer_ (0),
      speed_left_ (0.0),
      speed_right_ (0.0),
      trailer_angle_ (0.0)
  {
  }
  
  
  void DifferentialTrailerDrive::
  setSpeed (double wl, double wr)
  {
    speed_left_ = wl;
    speed_right_ = wr;
  }
  
  
  void DifferentialTrailerDrive::
  integrate (double dt)
  {
    if (( ! tractor_) || ( ! trailer_)) {
      return;
    }
    
    double const dl (radius_left_ * speed_left_);
    double const dr (radius_right_ * speed_right_);
    double const vtrans ((dl + dr) / 2.0);
    double const vrot ((dr - dl) / wheel_base_);
    double const dphi (-dt
		       * (vtrans * sin (trailer_angle_)
			  + vrot * (hitch_offset_ * cos (trailer_angle_) + 1.0))
		       / trailer_arm_);
    
    tractor_->motion_.Add (vtrans * tractor_->motion_.Costheta(),
			   vtrans * tractor_->motion_.Sintheta(),
			   dt * vrot);
    
    trailer_angle_ += dphi;
    trailer_->mount_.Set (-hitch_offset_, 0.0, 0.0);
    trailer_->motion_.Set (-trailer_arm_ * cos (trailer_angle_),
			   -trailer_arm_ * sin (trailer_angle_),
			   trailer_angle_);
  }
  
}
