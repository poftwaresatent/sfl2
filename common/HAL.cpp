/* 
 * Copyright (C) 2005
 * Swiss Federal Institute of Technology, Lausanne. All rights reserved.
 * 
 * Author: Roland Philippsen <roland dot philippsen at gmx dot net>
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


#include "HAL.hpp"
#include "RobotServer.hpp"
#include "Lidar.hpp"
#include "Random.hpp"
#include "pdebug.hpp"
#include <sfl/util/Frame.hpp>
#include <sfl/util/numeric.hpp>
#include <iostream>
#include <sys/time.h>


using namespace sfl;
using namespace boost;


namespace npm {
  
  
  HAL::
  HAL(RobotServer * owner)
    : m_owner(owner),
      m_noisy_odometry(false),
      m_noisy_scanners(false)
  {
    for(size_t ii(0); ii < 3; ++ii){
      m_current_speed[ii] = 0;
      m_wanted_speed[ii] = 0;
    }
  }
  
  
  int HAL::
  time_get(struct timespec * stamp)
  {
    struct timeval tv;
    int res(gettimeofday(&tv, 0));
    if(res != 0)
      return -1;
    TIMEVAL_TO_TIMESPEC(&tv, stamp);
    return 0;
  }


  int HAL::
  odometry_set(double x, double y, double theta,
	       double sxx, double syy, double stt,
	       double sxy, double sxt, double syt)
  {
    if(m_owner->GetTrueTrajectory().empty())
      m_owner->InitializePose(Frame(x, y, theta));
    else
      m_owner->AddPose(shared_ptr<Frame>(new Frame(x, y, theta)));
    return 0;
  }
  
  
  int HAL::
  odometry_get(struct timespec * stamp,
	       double * x, double * y, double * theta,
	       double * sxx, double * syy, double * stt,
	       double * sxy, double * sxt, double * syt)
  {
    int res(time_get(stamp));
    if(res != 0)
      return res;
    
    const sfl::Frame * pose;
    if(m_noisy_odometry){
      pose = m_owner->GetNoisyPose();
      PVDEBUG("noisy odometry baby!\n");
      if( ! pose){
	pose = & m_owner->GetTruePose();
	PVDEBUG("BUT IT'S BROKEN!\n");
      }
    }
    else
      pose = & m_owner->GetTruePose();
    
    pose->Get(*x, *y, *theta);
    *sxx = 1;
    *syy = 1;
    *stt = 1;
    *sxy = 0;
    *sxt = 0;
    *syt = 0;
    return 0;
  }
  
  
  int HAL::
  speed_set(double qdl, double qdr)
  {
    m_wanted_speed[0] = qdl;
    m_wanted_speed[1] = qdr;
    m_wanted_speed[2] = 0;
    return 0;
  }
  
  
  int HAL::
  speed_get(double * qdl, double * qdr)
  {
    * qdl = m_current_speed[0];
    * qdr = m_current_speed[1];
    
    if(m_noisy_odometry){
      PVDEBUG("noisy speed baby!\n");
      *qdl *= Random::Uniform(0.95, 1.05);
      *qdr *= Random::Uniform(0.95, 1.05);
    }
    
    return 0;
  }
  
  
  int HAL::
  scan_get(int channel, double * rho, size_t * rho_len,
	   struct ::timespec * t0, struct ::timespec * t1)
  {
    boost::shared_ptr<const Lidar> lidar(m_owner->GetLidar(channel));
    if( ! lidar)
      return -42;
    *rho_len = minval(*rho_len, lidar->nscans);
    
    if(m_noisy_scanners){
      PVDEBUG("noisy scanners baby!\n");
      for(size_t is(0); is < *rho_len; ++is)
	rho[is] = lidar->GetNoisyRho(is);
    }
    else
      for(size_t is(0); is < *rho_len; ++is)
	rho[is] = lidar->GetTrueRho(is);
    
    *t0 = lidar->GetT0();
    *t1 = lidar->GetT1();
    return 0;
  }
  
  
  void HAL::
  speed_set(double vx, double vy, double omega)
  {
    m_wanted_speed[0] = vx;
    m_wanted_speed[1] = vy;
    m_wanted_speed[2] = omega;
  }
  
  
  void HAL::
  speed_get(double & vx, double & vy, double & omega)
  {
    vx = m_wanted_speed[0];
    vy = m_wanted_speed[1];
    omega = m_wanted_speed[2];

    if(m_noisy_odometry){
      PVDEBUG("noisy speed baby!\n");
      vx *= Random::Uniform(0.95, 1.05);
      vy *= Random::Uniform(0.95, 1.05);
      omega *= Random::Uniform(0.95, 1.05);
    }
  }
  
  
  void HAL::
  UpdateSpeeds()
  {
    for(size_t ii(0); ii < 3; ++ii)
      m_current_speed[ii] = m_wanted_speed[ii];
  }
  
  
  void HAL::
  EnableOdometryNoise()
  {
    m_noisy_odometry = true;
  }
  
  
  void HAL::
  DisableOdometryNoise()
  {
    m_noisy_odometry = false;
  }
  
  
  void HAL::
  EnableScannerNoise()
  {
    m_noisy_scanners = true;
  }
  
  
  void HAL::
  DisableScannerNoise()
  {
    m_noisy_scanners = false;
  }
  
  
}
