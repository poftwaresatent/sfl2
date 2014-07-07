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

#include "AliceProcess.hpp"
#include "DifferentialDrive.hpp"
#include "RevoluteServo.hpp"
#include "RayDistanceSensor.hpp"
#include <boost/bind.hpp>

#include <cmath>


namespace npm2 {
  
  
  AliceProcess::
  AliceProcess (string const & name)
    : Process (name),
      drive_ (0),
      servo_ (0),
      sensor_ (0)
  {
    reflectSlot ("drive", &drive_);
    reflectSlot ("servo", &servo_);
    reflectSlot ("sensor", &sensor_);
    reflectCallback <string> ("attach", true, boost::bind (&AliceProcess::attach, this, _1));
  }
  
  
  bool AliceProcess::
  attach (string const & base_name)
  {
    Object * obj (Object::registry.find (base_name));
    if ( ! obj) {
      printf ("* cannot find Object %s\n", base_name.c_str());
      return false;
    }
    
    drive_ = obj->find <DifferentialDrive> (name + "_drive");
    if ( ! drive_) {
      printf ("* cannot find drive %s%s\n", name.c_str(), "_drive");
      return false;
    }
    
    servo_ = obj->find <RevoluteServo> (name + "_servo");
    if ( ! servo_) {
      printf ("* cannot find servo %s%s\n", name.c_str(), "_servo");
      return false;
    }

    sensor_ = obj->find <RayDistanceSensor> (name + "_sensor");
    if ( ! sensor_) {
      printf ("* cannot find sensor %s%s\n", name.c_str(), "_sensor");
      return false;
    }
    
    return true;
  }
  
  
  AliceProcess::state_t AliceProcess::
  init (ostream & erros)
  {
    if (( ! drive_) || ( ! servo_) || ( ! sensor_)) {
      erros << "AliceProcess " << name << " needs a drive, servo, and sensor\n";
      return FAILED;
    }
    return RUNNING;
  }
  
  
  AliceProcess::state_t AliceProcess::
  run (double timestep, ostream & erros)
  {
    drive_->setSpeed (0.02, 0.04);
    
    static double amp (5.0 * M_PI / 180.0);
    static double omg (2.0 * M_PI / 5.0);
    static size_t count (0);
    servo_->setAngle (amp * cos (omg * (count++) * timestep));
    
    return RUNNING;
  }
  
}
