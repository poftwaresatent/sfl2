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

#include "Object.hpp"
#include "Sensor.hpp"


namespace npm2 {

  
  Object::
  Object ()
    : parent_ (0)
  {
  }
  
  
  Object::
  ~Object()
  {
  }
  
  
  void Object::
  setParent (Object * obj)
  {
    if (parent_) {
      parent_->children_.erase (this);
    }
    parent_ = obj;
    if (parent_) {
      parent_->children_.insert (this);
    }
  }
  
  
  void Object::
  updateTransform ()
  {
    global_ = motion_;
    mount_.To (global_);
    if (parent_) {
      parent_->global_.To (global_);
    }
    body_.transformTo (global_);
    
    for (children_t::iterator ic (children_.begin()); ic != children_.end(); ++ic) {
      (*ic)->updateTransform();
    }
  }
  
  
  void Object::
  updateSensor (Sensor * sensor)
  {
    sensor->sensorUpdate (body_);
    for (children_t::iterator ic (children_.begin()); ic != children_.end(); ++ic) {
      (*ic)->updateSensor (sensor);
    }
  }
  
}
