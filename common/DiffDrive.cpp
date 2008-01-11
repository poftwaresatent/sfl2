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


#include "DiffDrive.hpp"
#include "HAL.hpp"
#include <sfl/util/Frame.hpp>
#include <sfl/util/numeric.hpp>


using namespace sfl;
using namespace boost;


namespace npm {


  DiffDrive::
  DiffDrive(shared_ptr<HAL> hal, double _wheelbase, double _wheelradius)
    : Drive(hal), wheelbase(_wheelbase), wheelradius(_wheelradius)
  {
  }


  shared_ptr<Frame> DiffDrive::
  ComputeNextPose(const Frame & current, double timestep) const
  {
    shared_ptr<Frame> result(new Frame(current));
  
    double qd[2];
    size_t len(2);
    const int status(m_hal->speed_get(qd, &len));
    if ((0 != status) || (2 != len))
      return result;
    
    // actuator speed -> global speed
    double dl    = qd[0] * wheelradius;
    double dr    = qd[1] * wheelradius;
    double v     = (dl + dr) / 2;
    double omega = (dr - dl) / wheelbase;
  
    // local kinematics
    double dtheta = omega * timestep;
    double dx, dy;
    double R;
    if(absval(dtheta) > epsilon){
      // use circular movement
      R = v / omega;
      dx = R * sin(dtheta);
      dy = R * (1 - cos(dtheta));
    }
    else {
      // approximate with linear movement
      R = v * timestep;
      dx = R * cos(0.5 * dtheta);
      dy = R * sin(0.5 * dtheta);
    }
  
    // rotate and transform to global frame
    current.RotateTo(dx, dy);
    result->Add(dx, dy, dtheta);
  
    return result;
  }

}
