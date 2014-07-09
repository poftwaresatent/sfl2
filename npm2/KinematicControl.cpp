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
#include <sfl/util/numeric.hpp>
#include <cmath>


namespace npm2 {
  
  
  KinematicControl::
  KinematicControl (string const & name)
    : Process (name),
      kr_ (3.0),
      kd_ (-1.5),
      kg_ (8.0),
      drive_ (0),
      vtrans_max_ (-1.0),
      vrot_max_ (-1.0)
  {
    reflectParameter ("goal", &goal_);
    reflectParameter ("kr", &kr_);
    reflectParameter ("kd", &kd_);
    reflectParameter ("kg", &kg_);
    reflectSlot ("drive", &drive_);
    reflectParameter ("vtrans_max", &vtrans_max_);
    reflectParameter ("vrot_max", &vrot_max_);
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
    double const dx (goal_.X() - obj->getGlobal().X());
    double const dy (goal_.Y() - obj->getGlobal().Y());

    double rho (sqrt (dx * dx + dy * dy));
    double eps, gamma, delta;
    if (fabs (rho) > 1.0e-5) {	// a discontinuous way of avoiding numerical instability of atan2
      eps = atan2 (dy, dx);
      gamma = mod2pi (eps - obj->getGlobal().Theta());
      delta = mod2pi (goal_.Theta() - gamma - obj->getGlobal().Theta());
    }
    else {
      rho = 0.0;
      eps = 0.0;
      gamma = mod2pi (goal_.Theta() - obj->getGlobal().Theta());
      delta = 0.0;
    }
    
    double vtrans (kr_ * rho);
    double vrot (kg_ * gamma + kd_ * delta);
    double strans (1.0);
    if ((0.0 < vtrans_max_) && (fabs (vtrans) > vtrans_max_)) {
      strans = vtrans_max_ / fabs (vtrans);
    }
    double srot (1.0);
    if ((0.0 < vrot_max_) && (fabs (vrot) > vrot_max_)) {
      srot = vrot_max_ / fabs (vrot);
    }
    double const sat (srot < strans ? srot : strans);
    
    // printf ("%6.4f  %6.4f  %6.4f   %6.4f  %6.4f    %6.4f  %6.4f\n",
    // 	    strans, srot, sat, vtrans, vrot, sat * vtrans, sat * vrot);
    
    drive_->setSpeed (sat * vtrans, sat * vrot);
    return RUNNING;
  }
  
}
