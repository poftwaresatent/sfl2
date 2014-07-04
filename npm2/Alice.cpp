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

#include "Alice.hpp"
#include "DifferentialDrive.hpp"
#include "RevoluteServo.hpp"
#include "RayDistanceSensor.hpp"

#include <cmath>


namespace npm2 {
  
  
  Alice::
  Alice (string const & name)
    : Process (name),
      drive_ (0),
      servo_ (0),
      sensor_ (0)
  {
    reflectSlot ("drive", &drive_);
    reflectSlot ("servo", &servo_);
    reflectSlot ("sensor", &sensor_);
  }
  
  
  Alice::state_t Alice::
  run (double timestep, ostream & erros)
  {
    if (( ! drive_) || ( ! servo_) || ( ! sensor_)) {
      erros << "Alice " << name << " needs a drive, servo, and sensor\n";
      return FAILED;
    }
    
    drive_->setSpeed (0.02, 0.04);
    
    static double amp (5.0 * M_PI / 180.0);
    static double omg (2.0 * M_PI / 5.0);
    static size_t count (0);
    servo_->setAngle (amp * cos (omg * (count++) * timestep));
    
    return RUNNING;
  }
  
}
