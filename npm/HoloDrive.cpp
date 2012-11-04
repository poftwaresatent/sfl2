/* 
 * Copyright (C) 2004
 * Swiss Federal Institute of Technology, Lausanne. All rights reserved.
 * 
 * Developed at the Autonomous Systems Lab.
 * Visit our homepage at http://asl.epfl.ch/
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


#include "HoloDrive.hpp"
#include "HAL.hpp"
#include <sfl/util/Frame.hpp>


using namespace sfl;
using namespace boost;


namespace npm {


  HoloDrive::
  HoloDrive(shared_ptr<HAL> hal, double _axislength)
    : Drive(hal), axislength(_axislength)
  {
  }


  shared_ptr<Frame> HoloDrive::
  ComputeNextPose(const Frame & current, double timestep) const
  {
    shared_ptr<Frame> result(new Frame(current));
    double qd[3];
    size_t len(3);
    if ((0 != m_hal->speed_get(qd, &len)) || (3 != len))
      return result;
    double dx(qd[0] * timestep);
    double dy(qd[1] * timestep);
    double dtheta(qd[2] * timestep);
    current.RotateTo(dx, dy);
    result->Add(dx, dy, dtheta);
    return result;
  }
  
}