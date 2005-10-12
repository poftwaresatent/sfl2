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


#ifndef SUNFLOWER_HOLODRIVE_HPP
#define SUNFLOWER_HOLODRIVE_HPP


#include <sfl/api/Drive.hpp>
#include <sfl/util/Frame.hpp>
#include <string>


namespace sfl {

  /**
     \brief Holonomic drive actuator.

     \todo DEPRECATE NOW!!!
  */
  class HoloDrive:
    public Drive
  {
  public:
    HoloDrive(const std::string & name,
	      double axislength);
    
    /** Set the speeds. */
    void SetSpeed(double vx,
		  double vy,
		  double omega);
    
    /** Implements Drive. */
    boost::shared_ptr<Frame> NextPose(const Frame & pose, double timestep);
    
    inline double AxisLength() const;
    
    /** \return The last value returned by NextPose(). */
    inline const Frame & PoseCache() const;
    
    
private:
    /** length of axes when drawing the drive position */
    double _axislength;
    
    /** Current speed along x [m/s] */
    double _vx;
    
    /** Current speed along y [m/s] */
    double _vy;
    
    /** Current rotational speed [rad/s] */
    double _omega;
    
    /** Last value returned from NextPose(). */
    Frame _pose_cache;
  };
  
  
  double HoloDrive::
  AxisLength()
    const
  {
    return _axislength;
  }
  
  
  const Frame & HoloDrive::
  PoseCache()
    const
  {
    return _pose_cache;
  }

}

#endif // SUNFLOWER_HOLODRIVE_HPP
