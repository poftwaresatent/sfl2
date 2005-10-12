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


#ifndef SUNFLOWER_DRIVE_HPP
#define SUNFLOWER_DRIVE_HPP


#include <boost/shared_ptr.hpp>


namespace sfl {
  
  
  class Frame;
  
  
  /**
     Pure abstract actuator class.
  */
  class Drive
  {
  public:
    virtual ~Drive() { }
  
    /**
       Kinematic model, computes the displacement of the robot when
       moving with the current actuator commands during a certain time
       period. Returns a new Frame instance.
       
       \note This method is not declared const such that subclasses
       can implement e.g. caching.
    */
    virtual boost::shared_ptr<Frame> NextPose(const Frame & pose,
					      double timestep) = 0;
  };
  
}

#endif // SUNFLOWER_DRIVE_HPP
