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


#ifndef SUNFLOWER_ODOMETRY_HPP
#define SUNFLOWER_ODOMETRY_HPP


#include <sfl/api/Pose.hpp>
#include <sfl/api/Timestamp.hpp>
#include <boost/shared_ptr.hpp>
#include <map>


namespace sfl {
  
  
  class HALProxy;
  
  
  /**
     Updating the position of the robot based on the wheel speeds is
     called odometry (or <em>ded</em>-reckoning, the short form of
     <em>deduced</em> reckoning... unfortunately often misspelled
     "dead-reckoning"). In sunflower, this is done in the HAL, and
     this class provides:
     
     <ul>
       <li> Access to the HAL odometry through a C++ interface. </li>
       <li> Pose history, i.e. for on-the-fly localization. </li>
     </ul>
     
     \todo Limit the size of the pose history, and use a more
     efficient implementation (e.g. fixed-length array instead
     of a STL map).
  */
  class Odometry
  {
  public:
    Odometry(HALProxy * hal_proxy);
    
    /**
       Initialize history with a pose in world frame. This clears any
       previous pose history and sets the HAL odometry to the wanted
       pose. (Also puts the pose as the only entry into the pose
       history).
       
       \return 0 on success.
    */
    int Init(/** initial position, will be copied into the history */
	     const Pose & pose,
	     /** if non-zero, debug messages are written to dbgos */
	     std::ostream * dbgos = 0);
    
    /**
       Update the pose history using the most recent pose. This reads
       the odometry from HAL and appends a copy to the pose history.
       
       \return 0 on success.
    */
    int Update(/** if non-zero, debug messages are written to dbgos */
	       std::ostream * dbgos = 0);
    
    /**
       \return Current (most recent) pose in world frame.
       
       \note Does not call Odometry::Update(), a dedicated update
       thread should take care of that.
    */
    const Pose & Get() const;
    
    /**
       \return If available, the Pose in world frame at a given
       time. 0 otherwise.

       \todo Create a way of retrieving information about which
       timestamps are available.
    */
    const Pose * Get(const Timestamp & t) const;
    
    /**
       Set the current pose in world frame. This sets the HAL odometry
       and copies the provided pose to the history.
       
       \return 0 on success.
    */
    int Set(const Pose & pose);
    
    
  protected:
    typedef std::map<Timestamp, boost::shared_ptr<Pose> > history_t;
    history_t m_history;
    
    HALProxy * m_hal_proxy;
  };
  
}

#endif // SUNFLOWER_ODOMETRY_HPP
