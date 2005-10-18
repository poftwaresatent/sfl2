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


#ifndef SUNFLOWER_DIFFDRIVE_HPP
#define SUNFLOWER_DIFFDRIVE_HPP


#include <sfl/api/Drive.hpp>
#include <sfl/util/Frame.hpp>


namespace sfl {
  
  
  class HALProxy;
  
  
  /**
     Differential drive actuator.
  */
  class DiffDrive:
    public Drive
  {
  public:
    DiffDrive(HALProxy * hal_proxy,
	      double wheelbase,
	      double wheelradius);
    
    
    /**
       Set the wheel speeds [rad/s].
       \return 0 on success.
    */
    int SetSpeed(double left, double right);
    
    /** Implements Drive::NextPose(). */
    virtual boost::shared_ptr<Frame> NextPose(const Frame & pose,
					      double timestep);
    
    double WheelBase() const { return m_wheelbase; }
    double WheelRadius() const { return m_wheelradius; }
    double Qdl() const { return m_qdl; }
    double Qdr() const { return m_qdr; }
    
    /** \return The last value returned by NextPose(). */
    const Frame & PoseCache() const { return m_pose_cache; }
    
    
  protected:
    HALProxy * m_hal_proxy;
    
    /** distance between wheel contact points [m] */
    const double m_wheelbase;
    
    /** radius of drive wheels [m] */
    const double m_wheelradius;
    
    /** current left wheel speed [rad/s] */
    double m_qdl;
    
    /** Current right wheel speed [rad/s] */
    double m_qdr;
    
    /** Last value returned from NextPose(). */
    Frame m_pose_cache;
  };
  
}

#endif // SUNFLOWER_DIFFDRIVE_HPP
