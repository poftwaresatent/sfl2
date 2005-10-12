/* 
 * Copyright (C) 2004
 * Swiss Federal Institute of Technology, Lausanne. All rights reserved.
 * 
 * Author: Roland Philippsen <roland.philippsen@gmx.net>
 *         Autonomous Systems Lab <http://asl.epfl.ch/>
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


#ifndef SUNFLOWER_MOTIONCONTROLLER_HPP
#define SUNFLOWER_MOTIONCONTROLLER_HPP


#include <sfl/util/Frame.hpp>
#include <sfl/util/numeric.hpp>


namespace sfl {
  
  
  class RobotModel;
  
  
  /**
     Encapsulates the motion controller. Used to control the robot's
     movement.
     
     \todo Use hal_speed_get() instead of relying on cached speed
     values.
  */
  class MotionController
  {
  public:
    MotionController(const RobotModel & robotModel);
    
    virtual ~MotionController() { }
    
    
    /**
       Template method for determining the next motion command. The
       speeds are set through ProposeSpeed() or
       ProposeActuators(). Applies kinodynamic limits first, then
       calls the virtual SendMotorCommand() with the resulting
       velocities.
       
       \note Using a separate SendMotorCommand() method allows
       subclasses to override the way how this information is
       used. This is useful for simulation or debugging.
       
       \todo ...SendMotorCommand() is kind of redundant with the
       HALProxy mechanism...
       
       \return The return value of the call to SendMotorCommand(),
       which must be 0 for success.
    */
    int Update(/** if non-zero, debug messages are written to dbgos */
	       std::ostream * dbgos = 0);
    
    /**
       Set the (global) speed to be applied during the next Update().
    */
    void ProposeSpeed(double sd, double thetad);

    /**
       Get the current (global) speed. These are the actual values
       that resulted from the previous Update().
    */
    void GetSpeed(double & sd, double & thetad) const;

    /**
       Set the actuator speeds to be applied during the next
       Update(). This is more direct than ProposeSpeed() and is used
       by sunflower's obstacle avoidance.
    */
    void ProposeActuators(double qdLeft, double qdRight);

    /**
       Get the current actuator speeds. These are the actual values
       that resulted from the previous Update().
    */
    void GetActuators(double & qdLeft, double & qdRight) const;
    
    
  protected:
    const double _qdMax;
    const double _sdMax;
    const double _thetadMax;
    const double _deltaQdMax;
    const double _qdStoppable;
    
    const RobotModel & _robotModel;
    
    double _proposedQdl, _proposedQdr;
    double _actualQdl, _actualQdr;
    
    /**
       Called from within Update() template method, passing the freshly
       calculated _actualQdl, _actualQdr as parameters.
       
       \note This method must be overridden by subclasses to implement
       actual motor control.
       
       \return 0 on success.
    */
    virtual int SendMotorCommand(double qdl, double qdr) = 0;
  };

}

#endif // SUNFLOWER_MOTIONCONTROLLER_HPP
