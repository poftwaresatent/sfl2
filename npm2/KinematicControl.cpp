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

#include "KinematicControl.hpp"
#include <cmath>


namespace npm2 {
  
  
  KinematicControl::
  KinematicControl (string const & name)
    : Process (name),
      kv_ (3.0),
      ka_ (8.0),
      kb_ (-1.5),
      drive_ (0)
  {
    reflectParameter ("goal", &goal_);
    reflectParameter ("kv", &kv_);
    reflectParameter ("ka", &ka_);
    reflectParameter ("kb", &kb_);
    reflectSlot ("drive", &drive_);
  }
  
  
  KinematicControl::state_t KinematicControl::
  init (ostream & erros)
  {
    if ( ! drive_) {
      erros << "KinematicControl " << name << " needs a drive\n";
      return FAILED;
    }
    return RUNNING;
  }
  
  
  KinematicControl::state_t KinematicControl::
  run (double timestep, ostream & erros)
  {
    Object const * obj (drive_->getParent());
    double const ex (goal_.X() - obj->getGlobal().X());
    double const ey (goal_.Y() - obj->getGlobal().Y());
    double const eth (goal_.Theta() - obj->getGlobal().Theta());
    
    // should also include damping...
    drive_->setSpeed (kv_ * sqrt (ex * ex + ey * ey),
		      (ka_ - kb_) * atan2 (ey, ex) + ka_ * eth);
  }
  
}
