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


#include "GenomBridge.hpp"
#include "LAAS.hpp"
#include "phelp.hpp"
//#include <npm/common/Lidar.hpp>
#include <npm/common/HAL.hpp>
#include <sfl/util/Pthread.hpp>
//#include <sfl/api/RobotModel.hpp>
#include <sfl/api/Scanner.hpp>
#include <sfl/util/Frame.hpp>
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


GenomBridge::
GenomBridge()
  : m_pompos(poster::create(pomPosterName, sizeof(POM_POS))),
    m_cartspeed(poster::find(speedrefPosterName, sizeof(GENPOS_CART_SPEED))),
    m_goal(poster::create(goalPosterName, sizeof(SFL_GOAL))),
    m_curspeed(poster::create(curspeedPosterName, sizeof(SFL_CURSPEED))),
    m_scanpolar(poster::create(scannerPosterName,
			       sizeof(SICK_SCANPOLAR_POSTER_STR)))
{
}


GenomBridge::
~GenomBridge()
{
  if(0 != m_goal)      poster::destroy(m_goal);
  if(0 != m_curspeed)  poster::destroy(m_curspeed);
  if(0 != m_scanpolar) poster::destroy(m_scanpolar);
  if(0 != m_pompos)    poster::destroy(m_pompos);
}


bool GenomBridge::
SetOdometry(double x, double y, double theta,
	    double sxx, double syy, double stt,
	    double sxy, double sxt, double syt)
{
  if(0 == m_pompos){
    m_pompos = poster::create(pomPosterName, sizeof(POM_POS));
    if(0 == m_pompos)
      return false;
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
  
  timeval t0;
  gettimeofday( & t0, NULL);
  pom_pos.pomTickDate  = static_cast<unsigned long long>(t0.tv_sec  * 1000);
  pom_pos.pomTickDate += static_cast<unsigned long long>(t0.tv_usec / 1000);
  
  return poster::write(m_pompos, &pom_pos, sizeof(POM_POS));
}


bool GenomBridge::
GetSpeedref(double & sd, double & thetad, int & numRef)
{
  if(0 == m_cartspeed){
    m_cartspeed = poster::find(speedrefPosterName, sizeof(GENPOS_CART_SPEED));
    if(0 == m_cartspeed)
      return false;
  }
  
  GENPOS_CART_SPEED speedRef;
  if( ! poster::read(m_cartspeed, &speedRef, sizeof(GENPOS_CART_SPEED))){
    m_cartspeed = 0;
    return false;
  }
  
  sd = speedRef.v;
  thetad = speedRef.w;
  numRef = speedRef.numRef;
  return true;
}


bool GenomBridge::
SetGoal(double x, double y, double theta,
	double dr, double dtheta, int via_goal)
{
  if(0 == m_goal){
    m_goal = poster::create(goalPosterName, sizeof(SFL_GOAL));
    if(0 == m_goal)
      return false;
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
  
  return poster::write(m_goal, &goal, sizeof(SFL_GOAL));
}


bool GenomBridge::
SetCurspeed(double sd, double thetad)
{
  if(0 == m_curspeed){
    m_curspeed = poster::create(curspeedPosterName, sizeof(SFL_CURSPEED));
    if(0 == m_curspeed)
      return false;
  }
  
  SFL_CURSPEED curspeed;
  curspeed.sd = sd;
  curspeed.thetad = thetad;
  
  timeval t0;
  gettimeofday( & t0, NULL);
  curspeed.timestamp  = static_cast<unsigned long long>(t0.tv_sec  * 1000);
  curspeed.timestamp += static_cast<unsigned long long>(t0.tv_usec / 1000);
  
  return poster::write(m_curspeed, &curspeed, sizeof(SFL_CURSPEED));
}


bool GenomBridge::
SetScan(const sfl::Scanner & scanner, HAL & hal)
{
  static SICK_SCANPOLAR_POSTER_STR polar;
  
  if(0 == m_scanpolar){
    m_scanpolar = poster::create(scannerPosterName,
				 sizeof(SICK_SCANPOLAR_POSTER_STR));
    if(0 == m_scanpolar)      
      return false;
  }
  
  if(scanner.nscans != polar.Header.np){
    SICK_MEASURES_HEADER_STR * header = &polar.Header;
    header->np = scanner.nscans;
    header->frame = SICK_SICK_FRAME;
    header->send_time = 0;
    header->rcv_time = 0;
    header->theta_min = scanner.phi0;
    header->dtheta = scanner.phirange;
    header->sickPomPos.date = 0;
    POM_EULER_V * sensorToMain = &header->sickPomPos.sensorToMain;
    sensorToMain->euler.yaw = scanner.mount->Theta();
    sensorToMain->euler.pitch = 0;
    sensorToMain->euler.roll = 0;
    sensorToMain->euler.x = scanner.mount->X();
    sensorToMain->euler.y = scanner.mount->Y();
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
  const int status(hal.scan_get(scanner.hal_channel, rho, &rho_len, &t0, &t1));
  if(0 != status){
    cerr << "scan_get() failed (" << status << ")\n";
    return false;
  }
  cerr << "GenomBridge::SetScan(): got " << rho_len << " sending "
       << polar.Header.np << "\n";
  
  polar.Header.sickPomPos.date = t1.tv_sec * 1000 + t1.tv_nsec / 1000000;
  polar.Header.rcv_time = polar.Header.sickPomPos.date;
  for(size_t ii(0); ii < rho_len; ++ii)
    polar.Rho[ii] = rho[ii];
  for(size_t ii(rho_len); ii < SICK_N_PT_MAX; ++ii)
    polar.Rho[ii] = 8;
  
  return poster::write(m_scanpolar, &polar, sizeof(SICK_SCANPOLAR_POSTER_STR));
}
