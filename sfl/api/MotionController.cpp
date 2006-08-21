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
#include "Timestamp.hpp"
#include "RobotModel.hpp"
#include "DiffDrive.hpp"
#include <sfl/util/pdebug.hpp>
#include <iostream>


#define PDEBUG PDEBUG_OFF
#define PVDEBUG PDEBUG_OFF


namespace sfl {


  MotionController::
  MotionController(const RobotModel & robotModel,
		   DiffDrive & drive):
    _qdMax(robotModel.QdMax()),
    _qddMax(robotModel.QddMax()),
    _sdMax(robotModel.SdMax()),
    _thetadMax(robotModel.ThetadMax()),
    _robotModel(robotModel),
    m_drive(drive),
    _proposedQdl(0),
    _proposedQdr(0),
    _actualQdl(0),
    _actualQdr(0)
  {
  }
  
  
  int MotionController::
  Update(double timestep, std::ostream * dbgos)
  {
    PDEBUG("dt: %g   curr: %g   %g\n", timestep, _actualQdl, _actualQdr);
    
    if(dbgos != 0)
      (*dbgos) << "INFO from MotionController::Update()\n"
	       << "  proposed: (" << _proposedQdl << ", "
	       << _proposedQdr << ")\n";
    
    // limit proposed actuator speeds to actuator dynamics
    const double deltaQdMax(timestep * _qddMax);
    const double qdlMax(boundval(- _qdMax, _actualQdl + deltaQdMax, _qdMax));
    const double qdlMin(boundval(- _qdMax, _actualQdl - deltaQdMax, _qdMax));
    const double qdrMax(boundval(- _qdMax, _actualQdr + deltaQdMax, _qdMax));
    const double qdrMin(boundval(- _qdMax, _actualQdr - deltaQdMax, _qdMax));
    PDEBUG("prop: %g   %g\n", _proposedQdl, _proposedQdr);
    _proposedQdl = boundval(qdlMin, _proposedQdl, qdlMax);
    _proposedQdr = boundval(qdrMin, _proposedQdr, qdrMax);
    PDEBUG("prop: %g   %g\n", _proposedQdl, _proposedQdr);
    
    // limit proposed actuator speeds to global dynamics
    double sd, thetad;
    _robotModel.Actuator2Global(_proposedQdl, _proposedQdr, sd, thetad);
    sd =     boundval( - _sdMax,     sd,     _sdMax);
    thetad = boundval( - _thetadMax, thetad, _thetadMax);
    _robotModel.Global2Actuator(sd, thetad, _actualQdl, _actualQdr);
    PDEBUG("act : %g   %g\n", _proposedQdl, _proposedQdr);
    
    // call actual motor control
    if(dbgos != 0)
      (*dbgos) << "  sending : (" << _actualQdl << ", " << _actualQdr << ")\n";
    
    return m_drive.SetSpeed(_actualQdl, _actualQdr);
  }
  
  
  void MotionController::
  ProposeSpeed(double sd, double thetad)
  {
    _robotModel.Global2Actuator(sd, thetad, _proposedQdl, _proposedQdr);
  }
  
  
  void MotionController::
  GetSpeed(double & sd, double & thetad)
    const
  {
    _robotModel.Actuator2Global(_actualQdl, _actualQdr, sd, thetad);
  }
  
  
  void MotionController::
  GetActuators(double & qdLeft, double & qdRight)
    const
  {
    qdLeft = _actualQdl;
    qdRight = _actualQdr;
  }
  
  
  void MotionController::
  ProposeActuators(double qdLeft, double qdRight)
  {
    _proposedQdl = qdLeft;
    _proposedQdr = qdRight;
  }

}
