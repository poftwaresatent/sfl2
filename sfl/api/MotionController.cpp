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
#include <iostream>


namespace sfl {


  MotionController::
  MotionController(const RobotModel & robotModel):
    _qdMax(robotModel.QdMax()),
    _sdMax(robotModel.SdMax()),
    _thetadMax(robotModel.ThetadMax()),
    _deltaQdMax(robotModel.Timestep() * robotModel.QddMax()),
    _qdStoppable(robotModel.Timestep() * robotModel.QddMax()),
    _robotModel(robotModel),
    _proposedQdl(0),
    _proposedQdr(0),
    _actualQdl(0),
    _actualQdr(0)
  {
  }
  
  
  int MotionController::
  Update(std::ostream * dbgos)
  {
    if(dbgos != 0)
      (*dbgos) << "INFO from MotionController::Update()\n"
	       << "  proposed: (" << _proposedQdl << ", "
	       << _proposedQdr << ")\n";

    // check if proposed actuator values violate actuator dynamics
    double qdlMax(minval(  _qdMax, _actualQdl + _deltaQdMax));
    double qdlMin(maxval(- _qdMax, _actualQdl - _deltaQdMax));
    double qdrMax(minval(  _qdMax, _actualQdr + _deltaQdMax));
    double qdrMin(maxval(- _qdMax, _actualQdr - _deltaQdMax));

    if(_proposedQdl > qdlMax)
      _proposedQdl = qdlMax;
    if(_proposedQdl < qdlMin)
      _proposedQdl = qdlMin;

    if(_proposedQdr > qdrMax)
      _proposedQdr = qdrMax;
    if(_proposedQdr < qdrMin)
      _proposedQdr = qdrMin;

    // check if proposed actuator values violate global dynamics
    double sd, thetad;
    _robotModel.Actuator2Global(_proposedQdl, _proposedQdr, sd, thetad);

    if(sd > _sdMax)
      sd = _sdMax;
    if(sd < - _sdMax)
      sd = - _sdMax;

    if(thetad > _thetadMax)
      thetad = _thetadMax;
    if(thetad < - _thetadMax)
      thetad = - _thetadMax;
  
    // load actualQd{l, r}
    _robotModel.Global2Actuator(sd, thetad, _actualQdl, _actualQdr);
  
    ///////////////////////////////
    // CALL ACTUAL MOTOR CONTROL //
    ///////////////////////////////
    if(dbgos != 0)
      (*dbgos) << "  sending : (" << _actualQdl << ", " << _actualQdr << ")\n";
    
    return SendMotorCommand(_actualQdl, _actualQdr);
  }
  

  void MotionController::
  ProposeSpeed(double sd,
	       double thetad)
  {
    _robotModel.Global2Actuator(sd, thetad, _proposedQdl, _proposedQdr);
  }


  void MotionController::
  GetSpeed(double & sd,
	   double & thetad)
    const
  {
    _robotModel.Actuator2Global(_actualQdl, _actualQdr, sd, thetad);
  }


  void MotionController::
  GetActuators(double & qdLeft,
	       double & qdRight)
    const
  {
    qdLeft = _actualQdl;
    qdRight = _actualQdr;
  }


  void MotionController::
  ProposeActuators(double qdLeft,
		   double qdRight)
  {
    _proposedQdl = qdLeft;
    _proposedQdr = qdRight;
  }

}
