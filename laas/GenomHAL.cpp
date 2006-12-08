/* 
 * Copyright (C) 2006
 * Swiss Federal Institute of Technology, Zurich. All rights reserved.
 * 
 * Author: Roland Philippsen <roland dot philippsen at gmx dot net>
 *         Autonomous Systems Lab <http://www.asl.ethz.ch/>
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


#include "GenomHAL.hpp"
#include "LAAS.hpp"
#include "phelp.hpp"
#include <npm/common/Lidar.hpp>
#include <sfl/util/Pthread.hpp>
#include <sfl/api/RobotModel.hpp>
#include <sfl/api/Scanner.hpp>
#include <iostream>
#include <sys/time.h>

extern "C" {
#include <pom/pomStruct.h>
#include <sick/sickStruct.h>
#include <sick/server/sickPosterLibStruct.h>
#include <genPos/genPosStruct.h>
#include <sfl/sflStruct.h>
}

using namespace npm;
using namespace boost;
using namespace std;


static const char * pomPosterName("xcfwrapOdometry");
static const char * scannerPosterName("xcfwrapScan");
static const char * speedrefPosterName("sflSpeedRef");
static const char * goalPosterName("xcfwrapGoal");
static const char * curspeedPosterName("xcfwrapCurspeed");


GenomHAL::
GenomHAL(RobotServer * owner, LAAS * laas)
  : HAL(owner),
    m_laas(laas),
    m_mutex(sfl::Mutex::Create("GenomHAL"))
{
  if( ! m_mutex){
    cerr << "ERROR creating sfl::Mutex\n";
    exit(EXIT_FAILURE);
  }
  m_pompos = poster::create(pomPosterName, sizeof(POM_POS));
  m_cartspeed = poster::find(speedrefPosterName, sizeof(GENPOS_CART_SPEED));
  m_goal = poster::create(goalPosterName, sizeof(SFL_GOAL));
  m_curspeed = poster::create(curspeedPosterName, sizeof(SFL_CURSPEED));
  m_scanpolar = poster::create(scannerPosterName,
			       sizeof(SICK_SCANPOLAR_POSTER_STR));
}


GenomHAL::
~GenomHAL()
{
  cerr << "HELLO from ~GenomHAL()\n";
  
  sfl::Mutex::sentry sm(m_mutex);
  
  if(0 != m_goal)      poster::destroy(m_goal);
  if(0 != m_curspeed)  poster::destroy(m_curspeed);
  if(0 != m_scanpolar) poster::destroy(m_scanpolar);
  if(0 != m_pompos)    poster::destroy(m_pompos);
}


int GenomHAL::
odometry_set(double x, double y, double theta,
	     double sxx, double syy, double stt,
	     double sxy, double sxt, double syt)
{
  sfl::Mutex::sentry sm(m_mutex);
  
  int status(HAL::odometry_set(x, y, theta, sxx, syy, stt, sxy, sxt, syt));
  if(0 != status)
    return status;
  
  if(0 == m_pompos){
    m_pompos = poster::create(pomPosterName, sizeof(POM_POS));
    if(0 == m_pompos)
      return -123;
  }
  
  POM_POS pom_pos;
  POM_EULER_V *origin = &(pom_pos.mainToOrigin);
  origin->euler.x = x;
  origin->euler.y = y;
  origin->euler.yaw = theta;
  origin->var.vxx = sxx;
  origin->var.vyy = syy;
  origin->var.voo = stt;
  origin->var.vxy = sxy;
  origin->var.vox = sxt;
  origin->var.voy = syt;
  
  static int tick(0);
  pom_pos.pomTickDate = ++tick;
  
  if( ! poster::write(m_pompos, &pom_pos, sizeof(POM_POS)))
    return -124;
  
  return 0;
}


int GenomHAL::
speed_get(double * qdl, double * qdr)
{
  if( ! m_laas->m_robotModel){
    *qdl = 0;
    *qdr = 0;
    return 0;
  }
  
  sfl::Mutex::sentry sm(m_mutex);
  
  if(0 == m_cartspeed){
    m_cartspeed = poster::find(speedrefPosterName, sizeof(GENPOS_CART_SPEED));
    if(0 == m_cartspeed)
      return -123;
  }
  
  GENPOS_CART_SPEED speedRef;
  if( ! poster::read(m_cartspeed, &speedRef, sizeof(GENPOS_CART_SPEED))){
    m_cartspeed = 0;
    return -124;
  }
  
  m_laas->m_robotModel->Global2Actuator(speedRef.v, speedRef.w, *qdl, *qdr);
  
  // tell our superclass about the wanted speed to close the loop
  return HAL::speed_set(*qdl, *qdr);
}


int GenomHAL::
goal_set(double x, double y, double theta,
	 double dr, double dtheta, int via_goal)
{
  sfl::Mutex::sentry sm(m_mutex);
  
  if(0 == m_goal){
    m_goal = poster::create(goalPosterName, sizeof(SFL_GOAL));
    if(0 == m_goal)
      return -123;
  }
  
  SFL_GOAL goal;
  goal.x = x;
  goal.y = y;
  goal.theta = theta;
  goal.dr = dr;
  goal.dtheta = dtheta;
  goal.via_flag = via_goal;
  
  timeval t0;
  gettimeofday( & t0, NULL);
  goal.timestamp  = static_cast<unsigned long long>(t0.tv_sec  * 1000);
  goal.timestamp += static_cast<unsigned long long>(t0.tv_usec / 1000);
  
  if( ! poster::write(m_goal, &goal, sizeof(SFL_GOAL)))
    return -124;
  
  return 0;
}


void GenomHAL::
UpdateSpeeds()
{
  sfl::Mutex::sentry sm(m_mutex);
  
  if(0 == m_curspeed){
    m_curspeed = poster::create(curspeedPosterName, sizeof(SFL_CURSPEED));
    if(0 == m_curspeed)
      return;
  }
  
  HAL::UpdateSpeeds();
  
  double qdl, qdr;
  HAL::speed_get(&qdl, &qdr);	// don't use ours, would bite our bum!
  SFL_CURSPEED curspeed;
  if(m_laas->m_robotModel)
    m_laas->m_robotModel->Actuator2Global(qdl, qdr,
					  curspeed.sd, curspeed.thetad);
  else{
    curspeed.sd = 0;
    curspeed.thetad = 0;
  }
  
  timeval t0;
  gettimeofday( & t0, NULL);
  curspeed.timestamp  = static_cast<unsigned long long>(t0.tv_sec  * 1000);
  curspeed.timestamp += static_cast<unsigned long long>(t0.tv_usec / 1000);
  
  poster::write(m_curspeed, &curspeed, sizeof(SFL_CURSPEED));
}


void GenomHAL::
UpdateScan()
{
  static SICK_SCANPOLAR_POSTER_STR polar;
  
  sfl::Mutex::sentry sm(m_mutex);
  
  if(0 == m_scanpolar){
    m_scanpolar = poster::create(scannerPosterName,
				 sizeof(SICK_SCANPOLAR_POSTER_STR));
    if(0 == m_scanpolar)      
      return;
    
    SICK_MEASURES_HEADER_STR * header = &polar.Header;
    header->np = m_laas->m_front->nscans;
    header->frame = SICK_SICK_FRAME;
    header->send_time = 0;
    header->rcv_time = 0;
    header->theta_min = m_laas->m_front->GetScanner()->phi0;
    header->dtheta = m_laas->m_front->GetScanner()->phirange;
    header->sickPomPos.date = 0;
    POM_EULER_V * sensorToMain = &header->sickPomPos.sensorToMain;
    sensorToMain->euler.yaw = m_laas->m_front->mount->Theta();
    sensorToMain->euler.pitch = 0;
    sensorToMain->euler.roll = 0;
    sensorToMain->euler.x = m_laas->m_front->mount->X();
    sensorToMain->euler.y = m_laas->m_front->mount->Y();
    sensorToMain->euler.z = 0;
    bzero(&sensorToMain->var, sizeof(sensorToMain->var));
    bzero(&header->sickPomPos.mainToBase,
	  sizeof(header->sickPomPos.mainToBase));
    bzero(&header->sickPomPos.mainToOrigin,
	  sizeof(header->sickPomPos.mainToOrigin));
    bzero(&header->sickPomPos.VLocal,
	  sizeof(header->sickPomPos.VLocal));
  }
  
  double rho[SICK_N_PT_MAX];
  size_t rho_len(SICK_N_PT_MAX);
  struct ::timespec t0, t1;
  if(0 != scan_get(m_laas->m_front->GetScanner()->hal_channel, rho,
		   &rho_len, &t0, &t1)){
    cerr << "scan_get() failed\n";
    return;
  }
  
  polar.Header.sickPomPos.date = t1.tv_sec * 1000 + t1.tv_nsec / 1000000;
  polar.Header.rcv_time = polar.Header.sickPomPos.date;
  for(size_t ii(0); ii < rho_len; ++ii)
    polar.Rho[ii] = rho[ii];
  for(size_t ii(rho_len); ii < SICK_N_PT_MAX; ++ii)
    polar.Rho[ii] = 8;
  
  poster::write(m_scanpolar, &polar, sizeof(SICK_SCANPOLAR_POSTER_STR));
}


GenomHALFactory::
GenomHALFactory(LAAS * laas)
  : m_laas(laas)
{
}


GenomHAL * GenomHALFactory::
Create(npm::RobotServer * owner) const
{
  return new GenomHAL(owner, m_laas);
}
