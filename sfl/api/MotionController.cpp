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
#include "HAL.hpp"
#include <sfl/util/Mutex.hpp>
#include <iostream>


using namespace boost;
using namespace std;


namespace sfl {


  MotionController::
  MotionController(shared_ptr<const RobotModel> robotModel,
		   shared_ptr<HAL> hal,
		   shared_ptr<Mutex> mutex)
    : qdMax(robotModel->QdMax()), qddMax(robotModel->QddMax()),
      sdMax(robotModel->SdMax()), thetadMax(robotModel->ThetadMax()),
      m_robotModel(robotModel), m_hal(hal), m_mutex(mutex)
  {
  }
  
  
  int MotionController::
  Update(double timestep, ostream * dbgos)
  {
    Mutex::sentry sentry(m_mutex.get());
    int status(m_hal->speed_get( & m_currentQdl, & m_currentQdr));
    if(0 != status){
      if(dbgos != 0)
	(*dbgos) << "ERROR in MotionController::Update():\n"
		 << "  HAL::speed_get() returned " << status << "\n";
      return -1;
    }
    if(dbgos != 0)
      (*dbgos) << "INFO from MotionController::Update()\n"
	       << "  timestep: " << timestep << "\n"
	       << "  current:  (" << m_currentQdl << ", "
	       << m_currentQdr << ")\n"
	       << "  proposed: (" << m_proposedQdl << ", "
	       << m_proposedQdr << ")\n";
    
    // limit in actuator speed space
    const double dqd(timestep * qddMax);
    const double qdlMax(boundval(- qdMax, m_currentQdl + dqd, qdMax));
    const double qdlMin(boundval(- qdMax, m_currentQdl - dqd, qdMax));
    const double qdrMax(boundval(- qdMax, m_currentQdr + dqd, qdMax));
    const double qdrMin(boundval(- qdMax, m_currentQdr - dqd, qdMax));
    m_proposedQdl = boundval(qdlMin, m_proposedQdl, qdlMax);
    m_proposedQdr = boundval(qdrMin, m_proposedQdr, qdrMax);
    
    // limit in global speed space
    double sd, thetad;
    m_robotModel->Actuator2Global(m_proposedQdl, m_proposedQdr, sd, thetad);
    sd =     boundval( - sdMax,     sd,     sdMax);
    thetad = boundval( - thetadMax, thetad, thetadMax);
    m_robotModel->Global2Actuator(sd, thetad, m_wantedQdl, m_wantedQdr);
    
    // send it
    if(dbgos != 0)
      (*dbgos) << "  wanted:   (" << m_wantedQdl << ", "
	       << m_wantedQdr << ")\n";
    status = m_hal->speed_set(m_wantedQdl, m_wantedQdr);
    if(0 != status){
      if(dbgos != 0)
	(*dbgos) << "ERROR in MotionController::Update():\n"
		 << "  HAL::speed_set() returned " << status << "\n";
      return -2;
    }
    return 0;
  }
  
  
  void MotionController::
  ProposeSpeed(double sd, double thetad)
  {
    m_mutex->Lock();
    m_robotModel->Global2Actuator(sd, thetad, m_proposedQdl, m_proposedQdr);
    m_mutex->Unlock();
  }
  
  
  void MotionController::
  ProposeActuators(double qdLeft, double qdRight)
  {
    m_mutex->Lock();
    m_proposedQdl = qdLeft;
    m_proposedQdr = qdRight;
    m_mutex->Unlock();
  }
  
  
  void MotionController::
  GetCurrentGlob(double & sd, double & thetad) const
  {
    m_mutex->Lock();
    m_robotModel->Actuator2Global(m_currentQdl, m_currentQdr, sd, thetad);
    m_mutex->Unlock();
  }
  
  
  void MotionController::
  GetWantedGlob(double & sd, double & thetad) const
  {
    m_mutex->Lock();
    m_robotModel->Actuator2Global(m_wantedQdl, m_wantedQdr, sd, thetad);
    m_mutex->Unlock();
  }
  
  
  void MotionController::
  GetProposedGlob(double & sd, double & thetad) const
  {
    m_mutex->Lock();
    m_robotModel->Actuator2Global(m_proposedQdl, m_proposedQdr, sd, thetad);
    m_mutex->Unlock();
  }
  
  
  void MotionController::
  GetCurrentAct(double & qdLeft, double & qdRight) const
  {
    m_mutex->Lock();
    qdLeft = m_currentQdl;
    qdRight = m_currentQdr;
    m_mutex->Unlock();
  }
  
  
  void MotionController::
  GetWantedAct(double & qdLeft, double & qdRight) const
  {
    m_mutex->Lock();
    qdLeft = m_wantedQdl;
    qdRight = m_wantedQdr;
    m_mutex->Unlock();
  }
  
  
  void MotionController::
  GetProposedAct(double & qdLeft, double & qdRight) const
  {
    m_mutex->Lock();
    qdLeft = m_proposedQdl;
    qdRight = m_proposedQdr;
    m_mutex->Unlock();
  }
  
}
