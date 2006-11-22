/* 
 * Copyright (C) 2006
 * Swiss Federal Institute of Technology, Zurich. All rights reserved.
 * 
 * Developed at the Autonomous Systems Lab.
 * Visit our homepage at http://www.asl.ethz.ch/
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


#include "BicycleDrive.hpp"
#include "HAL.hpp"
#include <sfl/util/Frame.hpp>
#include <sfl/util/numeric.hpp>

using namespace sfl;
using namespace boost;


namespace npm {


  BicycleDrive::
  BicycleDrive(shared_ptr<HAL> hal, double _wheelbase, double _wheelradius)
    : Drive(hal), wheelbase(_wheelbase), wheelradius(_wheelradius)
  {
  }

  shared_ptr<Frame> BicycleDrive::
  ComputeNextPose(const Frame & current, double timestep) const
  {
      double v_trans, steer;

      //if I understood right HAL can manage whatever we get/set as the "speed"
      m_hal->speed_get(&v_trans, &steer);
    
      double dtheta = (v_trans / wheelbase) * tan(steer) * timestep;
 
      //it seems the robot drives along the y axis in nepumuk
      double dx = sin(dtheta) * v_trans * timestep;
      double dy = cos(dtheta) * v_trans * timestep;
      
      current.RotateTo(dx, dy);
      shared_ptr<Frame> result(new Frame(current));
      result->Add(dx, dy, dtheta);
      return result;
  }

}
