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
#include "Object.hpp"
#include <boost/bind.hpp>


namespace npm2 {
  
  
  Simulator::
  Simulator (string const & name)
    : fpplib::Configurable (name),
      world_ (0),
      timestep_ (0.1),
      state_ (PAUSE)
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
  
}
