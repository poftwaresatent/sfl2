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


#include "MotionController.hpp"
#include <sfl/api/HALProxy.hpp>


using sfl::maxval;
using sfl::minval;
using sfl::absval;
using sfl::epsilon;
using sfl::RobotModel;
using sfl::DiffDrive;
using sfl::HALProxy;


namespace expo {


  MotionController::
  MotionController(const std::string & name,
		   const RobotModel & robotModel,
		   DiffDrive & drive):
    sfl::MotionController(robotModel),
    m_drive(drive)
  {
  }
  
  
  int MotionController::
  SendMotorCommand(double qdl,
		   double qdr)
  {
    return m_drive.SetSpeed(qdl, qdr);
  }
  
  
  bool MotionController::
  Stoppable()
    const
  {
    return maxval(absval(_actualQdl), absval(_actualQdr)) < _qdStoppable;
  }
  
  
  bool MotionController::
  AlmostStraight()
    const
  {
    return absval(_actualQdl - _actualQdr) <= epsilon;
  }
  
  
  bool MotionController::
  Moving()
    const
  {
    return minval(absval(_actualQdl), absval(_actualQdr)) > epsilon;
  }
  
}
