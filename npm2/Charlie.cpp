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

#include "Charlie.hpp"
#include "DifferentialDrive.hpp"
#include "RayDistanceSensor.hpp"
#include <cmath>


namespace npm2 {
  
  
  Charlie::
  Charlie (string const & name)
    : RobotClient (name),
      drive_ (0),
      left_ (0),
      right_ (0),
      gain_ (0.3),
      offset_ (0.1)
  {
    reflectSlot ("drive", &drive_);
    reflectSlot ("left", &left_);
    reflectSlot ("right", &right_);
    reflectParameter ("gain", &gain_);
    reflectParameter ("offset", &offset_);
  }
  
  
  Charlie::state_t Charlie::
  init (ostream & erros)
  {
    if ( ! drive_) {
      erros << "Charlie " << name << " needs a drive\n";
      return FAILED;
    }
    if ( ! left_) {
      erros << "Charlie " << name << " needs a left sensor\n";
      return FAILED;
    }
    if ( ! right_) {
      erros << "Charlie " << name << " needs a right sensor\n";
      return FAILED;
    }
    return RUNNING;
  }
  
  
  Charlie::state_t Charlie::
  run (double timestep, ostream & erros)
  {
    double const wl (pow ((right_->distance_ - offset_) / right_->max_distance_, 2.0));
    double const wr (pow ((left_->distance_ - offset_) / left_->max_distance_, 2.0));
    drive_->setSpeed (gain_ * wl, gain_ * wr);
    
    printf ("Charlie %s  (%6.2f  %6.2f)  (%6.2f  %6.2f)\n",
	    name.c_str(), left_->distance_, right_->distance_, wl, wr);
  }
  
}
