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
#include <sfl/util/numeric.hpp>
#include <cmath>


using boost::shared_ptr;


namespace sfl {
  
  
  DiffDrive::
  DiffDrive(HAL * hal,
	    double wheelbase,
	    double wheelradius):
    m_hal(hal),
    m_wheelbase(wheelbase),
    m_wheelradius(wheelradius)
  {
    SetSpeed(0, 0);
  }
  
  
  int DiffDrive::
  SetSpeed(double left,
	   double right)
  {
    const int res(m_hal->speed_set(left, right));
    if(0 != res)
      return res;
    
    m_qdl = left;
    m_qdr = right;
    return 0;
  }
  
  
  shared_ptr<Frame> DiffDrive::
  NextPose(const Frame & pose,
	   double timestep)
  {
    // actuator speed -> global speed
    double dl    = m_qdl * m_wheelradius;
    double dr    = m_qdr * m_wheelradius;
    double v     = (dl + dr) / 2;
    double omega = (dr - dl) / m_wheelbase;
    
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
    pose.RotateTo(dx, dy);
    m_pose_cache.Set(pose);
    m_pose_cache.Add(dx, dy, dtheta);
    
    return shared_ptr<Frame>(new Frame(m_pose_cache));
  }
  
}
