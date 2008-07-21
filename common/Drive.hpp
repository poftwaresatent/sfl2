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


#ifndef NPM_DRIVE_HPP
#define NPM_DRIVE_HPP


#include <boost/shared_ptr.hpp>


namespace sfl {
  class Frame;
}

namespace npm {
  
  class HAL;
  
  /**
     Pure abstract actuator class. Somewhat overkill, but very useful
     for code that should be independent of the drive
     architecture. See DiffDrive and HoloDrive for (the currently
     only) subclasses.
  */
  class Drive
  {
  public:
    Drive(boost::shared_ptr<HAL> hal);
    virtual ~Drive();
    
    /** Compute the displacement of the robot when moving with the
	current speeds. Calls ComputeNextPose() and caches the result
	for PoseCache(). */
    boost::shared_ptr<sfl::Frame>
    NextPose(const sfl::Frame & current, double timestep);
    
    /** \return The last value returned by NextPose(). */
    boost::shared_ptr<const sfl::Frame> PoseCache() const;
    
    /**
       Transform the current motor commands into a Euclidean speed
       state expressed in the robot's local frame of reference.
       
       \note The default implementation always returns false.
       
       \return true if the computation succeeded (subclasses might
       not provide it)
    */
    virtual bool ComputeSpeedState(/** speed in [m/s] along the local X-axis */
				   double & xdot,
				   /** speed in [m/s] along the local Y-axis */
				   double & ydot,
				   /** rotational speed in [rad/s] */
				   double & thdot) const;
    
  protected:
    /** Implemented by subclasses. */
    virtual boost::shared_ptr<sfl::Frame>
    ComputeNextPose(const sfl::Frame & current, double timestep) const = 0;
    
    boost::shared_ptr<HAL> m_hal;
    boost::shared_ptr<sfl::Frame> m_cache;
  };

}

#endif // NPM_DRIVE_HPP
