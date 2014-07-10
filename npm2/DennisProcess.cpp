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

#include "DennisProcess.hpp"

namespace npm2 {
  
  
  DennisProcess::
  DennisProcess (string const & name)
    : Process (name)
  {
    reflectVectorParameter ("goals", &goals_);
    reflectSlot ("control", &control_);
  }
  
  
  DennisProcess::state_t DennisProcess::
  init (ostream & erros)
  {
    if ( ! control_) {
      erros << "DennisProcess needs a KinematicControl instance\n";
      return FAILED;
    }
    if (goals_.empty()) {
      erros << "DennisProcess needs some goals to work with\n";
      return FAILED;
    }
    
    current_ = 0;
    control_->setGoal (goals_[current_]);
    
    return RUNNING;
  }
  
  
  DennisProcess::state_t DennisProcess::
  run (double timestep, ostream & erros)
  {
    if ( ! goals_[current_].Reached (control_->drive_->getParent()->getGlobal(), true)) {
      return RUNNING;
    }
    current_ = (current_ + 1) % goals_.size();
    control_->setGoal (goals_[current_]);
    return RUNNING;
  }
  
}
