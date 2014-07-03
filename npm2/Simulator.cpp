/* 
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

#include "Simulator.hpp"
#include "Actuator.hpp"
#include "Sensor.hpp"
#include "RobotClient.hpp"
#include <boost/bind.hpp>


namespace npm2 {
  
  
  Simulator::
  Simulator (string const & name)
    : fpplib::Configurable (name),
      world_ (0),
      timestep_ (0.1),
      state_ (PAUSE),
      erros_ (cout)
  {
    reflectSlot ("world", &world_);
    reflectParameter ("timestep", &timestep_,
		      /** \todo parameter guards have unclear ownership */
		      new fpplib::StrictlyPositiveGuard <double> ());
    reflectCallback <string> ("state", true,
			      boost::bind (&Simulator::setState, this, _1));
  }
  
  
  bool Simulator::
  setState (string const & value)
  {
    if ("pause" == value || "PAUSE" == value) {
      state_ = PAUSE;
    }
    else if ("step" == value || "STEP" == value) {
      state_ = STEP;
    }
    else if ("run" == value || "RUN" == value) {
      state_ = RUN;
    }
    else {
      return false;
    }
    return true;
  }
  
  
  static void recurse_integrate (Object * obj, double dt)
  {
    Actuator * act (dynamic_cast <Actuator*> (obj));
    if (act) {
      act->integrate (dt);
    }
    for (Object::child_iterator_t ic(obj->childBegin()); ic != obj->childEnd(); ++ic) {
      recurse_integrate (*ic, dt);
    }
  }
  
  
  void Simulator::
  simulateActuators ()
  {
    recurse_integrate (world_, timestep_);
    world_->updateTransform ();
  }
  
  
  static void recurse_sense (Object const * world, Object * obj)
  {
    Sensor * sensor (dynamic_cast <Sensor*> (obj));
    if (sensor) {
      sensor->sensorReset ();
      world->updateSensor (sensor);
    }
    for (Object::child_iterator_t ic(obj->childBegin()); ic != obj->childEnd(); ++ic) {
      recurse_sense (world, *ic);
    }
  }
  
  
  void Simulator::
  simulateSensors ()
  {
    recurse_sense (world_, world_);
  }
  
  
  void Simulator::
  simulateProcesses ()
  {
    for (size_t ii(0); ii < RobotClient::registry.size(); ++ii) {
      RobotClient::registry.at(ii)->process (*this, erros_);
    }
  }
  
}
