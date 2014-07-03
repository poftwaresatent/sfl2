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

#include "RobotClient.hpp"
#include "Simulator.hpp"


namespace npm2 {
  
  
  RobotClient::registry_t RobotClient::registry;
  
  
  RobotClient::
  RobotClient (string const & name)
    : fpplib::Configurable (name),
      state_ (READY)
  {
    registry.add (name, this);
  }
  
  
  RobotClient::
  ~RobotClient ()
  {
    registry.remove (name, this);
  }
  
  
  RobotClient::state_t RobotClient::
  process (Simulator const & sim, ostream & erros)
  {
    switch (state_) {
    case READY:
      state_ = init (erros);
      break;
    case RUNNING:
      state_ = run (sim.timestep_, erros);
      break;
    case FAILED:
      state_ = recover (erros);
      break;
      // case DONE:
      //   do nothing
    }
    return state_;
  }
  
  
  RobotClient::state_t RobotClient::
  init (ostream & erros)
  {
    return RUNNING;
  }
  
  
  RobotClient::state_t RobotClient::
  recover (ostream & erros)
  {
    return FAILED;
  }
  
}
