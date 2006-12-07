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


template<typename PT>
POSTER_ID create_poster(const char * name)
{
  POSTER_ID id;
  if(posterCreate(const_cast<char *>(name), sizeof(PT), &id) != OK){
    perror(name);
    exit(EXIT_FAILURE);
  }
  
  char *tmp(reinterpret_cast<char *>(calloc(1, sizeof(PT))));
  if( ! tmp){
    cerr << "not enough mem to init poster " << name << "\n";
    exit(EXIT_FAILURE);
  }
  if(sizeof(PT) != posterWrite(id, 0, tmp, sizeof(PT))){
    cerr << "cannot init poster " << name << "\n";
    free(tmp);
    exit(EXIT_FAILURE);
  }
  free(tmp);
  
  return id;
}


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
  m_pompos = create_poster<POM_POS>("npm_pompos");
  m_scanpolar = create_poster<SICK_SCANPOLAR_POSTER_STR>("npm_scanpolar");
  m_curspeed = create_poster<SFL_CURSPEED>("npm_curspeed");
  m_goal = create_poster<SFL_GOAL>("npm_goal");
}


GenomHAL::
~GenomHAL()
{
  cerr << "HELLO from ~GenomHAL()\n";
  
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
  
  if(sizeof(POM_POS) != posterWrite(m_pompos, 0, &pom_pos, sizeof(POM_POS))){
    h2perror("posterWrite(m_pompos)");
    return -123;
  }
  
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
    int status(posterFind("sflSpeedRef", &m_cartspeed));
    if(ERROR == status){
      h2perror("finding sflSpeedRef");
      return -123;
    }
    int length;
    status = posterIoctl(m_cartspeed, FIO_GETSIZE, &length);
    if(ERROR == status){
      h2perror("ioctl on sflSpeedRef");
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
  if(sizeof(GENPOS_CART_SPEED) !=
     posterRead(m_cartspeed, 0, &speedRef, sizeof(GENPOS_CART_SPEED))){
    h2perror("posterRead(m_cartspeed)");
    m_cartspeed = 0;
    return -126;
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
  
  if(sizeof(SFL_GOAL) !=
     posterWrite(m_goal, 0, &goal, sizeof(SFL_GOAL))){
    h2perror("writing npm_goal");
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
  
  if(sizeof(SFL_CURSPEED) !=
     posterWrite(m_curspeed, 0, &curspeed, sizeof(SFL_CURSPEED)))
    h2perror("writing m_curspeed");
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
