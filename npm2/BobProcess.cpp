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

#include "BobProcess.hpp"
#include "DifferentialTrailerDrive.hpp"
#include <sfl/util/numeric.hpp>


namespace npm2 {
  
  
  BobProcess::
  BobProcess (string const & name)
    : Process (name),
      drive_ (0)
  {
    reflectSlot ("drive", &drive_);
  }
  
  
  BobProcess::state_t BobProcess::
  run (double timestep, ostream & erros)
  {
    if ( ! drive_) {
      erros << "BobProcess " << name << " is missing a drive\n";
      return FAILED;
    }
    
    double thref;
    if (drive_->getParent()->getGlobal().X() > drive_->getParent()->getGlobal().Y()) {
      if (drive_->getParent()->getGlobal().X() > - drive_->getParent()->getGlobal().Y()) {
	thref = 100.0 * M_PI / 180.0;
      }
      else {
	thref = 10.0 * M_PI / 180.0;
      }
    }
    else {
      if (drive_->getParent()->getGlobal().X() > - drive_->getParent()->getGlobal().Y()) {
	thref = 190.0 * M_PI / 180.0;
      }
      else {
	thref = -80.0 * M_PI / 180.0;
      }
    }
    double const dhead (mod2pi (thref - drive_->getParent()->getGlobal().Theta()));
    static double const dth (5.0 * M_PI / 180.0);
    if (fabs (dhead) <= dth) {
      drive_->setSpeed (0.02, 0.02);
    }
    else if (dhead > 0.0) {
      drive_->setSpeed (0.0, 0.02);
    }
    else {
      drive_->setSpeed (0.02, 0.0);
    }
    
    return RUNNING;
  }
  
}
