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
#include <sfl/util/Pthread.hpp>
#include <sfl/api/RobotModel.hpp>
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


GenomHAL::
GenomHAL(RobotServer * owner,
	 shared_ptr<const sfl::RobotModel> model)
  : HAL(owner),
    m_model(model),
    m_mutex(sfl::Mutex::Create("GenomHAL"))
{
  if( ! m_mutex){
    cerr << "ERROR creating sfl::Mutex\n";
    exit(EXIT_FAILURE);
  }
  STATUS status;
  status = posterCreate("npm_pompos", sizeof(POM_POS), &m_pompos);
  if(0 != status){
    cerr << "ERROR creating npm_pompos poster (" << status << ")\n";
    exit(EXIT_FAILURE);
  }
  status = posterCreate("npm_scanpolar", sizeof(SICK_SCANPOLAR_POSTER_STR),
			&m_scanpolar);
  if(0 != status){
    cerr << "ERROR creating npm_scanpolar poster (" << status << ")\n";
    exit(EXIT_FAILURE);
  }
  status = posterCreate("npm_curspeed", sizeof(SFL_CURSPEED), &m_curspeed);
  if(0 != status){
    cerr << "ERROR creating npm_curspeed poster (" << status << ")\n";
    exit(EXIT_FAILURE);
  }
  status = posterCreate("npm_goal", sizeof(SFL_GOAL), &m_goal);
  if(0 != status){
    cerr << "ERROR creating npm_goal poster (" << status << ")\n";
    exit(EXIT_FAILURE);
  }
}


GenomHAL::
~GenomHAL()
{
  sfl::Mutex::sentry sm(m_mutex);
  
  if(0 != m_goal)      posterDelete(&m_goal);
  if(0 != m_curspeed)  posterDelete(&m_curspeed);
  //  if(0 != m_cartspeed) posterDelete(&m_cartspeed);
  if(0 != m_scanpolar) posterDelete(&m_scanpolar);
  if(0 != m_pompos)    posterDelete(&m_pompos);
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
  
  status = posterWrite(m_pompos, 0, &pom_pos, sizeof(POM_POS));
  if(0 != status)
    return -123;
  
  return 0;
}


int GenomHAL::
speed_get(double * qdl, double * qdr)
{
  sfl::Mutex::sentry sm(m_mutex);
  
  if(0 == m_cartspeed){
    int status(posterFind("sflSpeedRef", &m_cartspeed));
    if(ERROR == status){
      cerr << "ERROR finding sflSpeedRef poster (" << status << ")\n";
      return -123;
    }
    int length;
    status = posterIoctl(m_cartspeed, FIO_GETSIZE, &length);
    if(ERROR == status){
      cerr << "ERROR poster ioctl on sflSpeedRef (" << status << ")\n";
      m_cartspeed = 0;
      return -124;
    }
    if(sizeof(GENPOS_CART_SPEED) != length){
      cerr << "ERROR sflSpeedRef has wrong length (" << length
	   << " instead of " << sizeof(GENPOS_CART_SPEED) << ")\n";
      m_cartspeed = 0;
      return -125;
    }
  }
  
  GENPOS_CART_SPEED speedRef;
  const int
    status(posterRead(m_cartspeed, 0, &speedRef, sizeof(GENPOS_CART_SPEED)));
  if(0 != status){
    cerr << "ERROR posterRead(m_cartspeed) failed (" << status << ")\n";
    m_cartspeed = 0;
    return -126;
  }
  
  m_model->Global2Actuator(speedRef.v, speedRef.w, *qdl, *qdr);
  
  // tell our superclass about the wanted speed to close the loop
  return HAL::speed_set(*qdl, *qdr);
}


int GenomHAL::
goal_set(double x, double y, double theta,
	 double dr, double dtheta, int via_goal)
{
  sfl::Mutex::sentry sm(m_mutex);
  
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
  
  const int status(posterWrite(m_goal, 0, &goal, sizeof(SFL_GOAL)));
  if(0 != status){
    cerr << "ERROR writing npm_goal poster (" << status << ")\n";
    return -123;
  }
  
  return 0;
}


void GenomHAL::
UpdateSpeeds()
{
  sfl::Mutex::sentry sm(m_mutex);
  
  HAL::UpdateSpeeds();
  
  double qdl, qdr;
  HAL::speed_get(&qdl, &qdr);	// don't use ours, would bite our bum!
  SFL_CURSPEED curspeed;
  m_model->Actuator2Global(qdl, qdr, curspeed.sd, curspeed.thetad);
  
  timeval t0;
  gettimeofday( & t0, NULL);
  curspeed.timestamp  = static_cast<unsigned long long>(t0.tv_sec  * 1000);
  curspeed.timestamp += static_cast<unsigned long long>(t0.tv_usec / 1000);
  
  const int
    status(posterWrite(m_curspeed, 0, &curspeed, sizeof(SFL_CURSPEED)));
  if(0 != status)
    cerr << "ERROR writing m_curspeed poster (" << status << ")\n";
}
