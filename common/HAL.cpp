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
#include "pdebug.hpp"
#include "NoiseModel.hpp"
#include <sfl/util/Frame.hpp>
#include <sfl/util/numeric.hpp>
#include <iostream>
#include <sys/time.h>


using namespace sfl;
using namespace boost;


namespace npm {
  
  
  HAL::
  HAL(RobotServer * owner, size_t _ndof)
    : ndof(_ndof),
      m_owner(owner),
      m_wanted_speed(new double[_ndof]),
      m_current_speed(new double[_ndof]),
      m_odometry_noise(0) // not all compilers seem to do this automatically
  {
    bzero(m_wanted_speed.get(), sizeof(double) * _ndof);
    bzero(m_current_speed.get(), sizeof(double) * _ndof);
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
    if(m_odometry_noise){
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
  speed_set(const double * qdot, size_t * qdot_len)
  {
    if (( ! qdot) || ( ! qdot_len)) {
      PDEBUG("qdot (%p) and qdot_len (%p) must not be null\n", qdot, qdot_len);
      return -1;
    }
    
    // it would be more efficient to use memcpy...

    PVDEBUG("ndof: %zu  qdot_len: %zu\n", ndof, *qdot_len);
    for (size_t ii(0); ii < ndof; ++ii) {
      if (ii >= *qdot_len) {
	for (/**/; ii < ndof; ++ii)
	  m_wanted_speed[ii] = 0;
	break;
      }
      PVDEBUG("  m_wanted_speed[%zu] = qdot[%zu] = %g\n", ii, ii, qdot[ii]);
      m_wanted_speed[ii] = qdot[ii];
    }
    *qdot_len = ndof;
    return 0;
  }
  
  
  int HAL::
  speed_get(double * qdot, size_t * qdot_len)
  {
    // it would be more efficient to use memcpy when we do not have
    // m_odometry_noise, but what the hey...
    
    if (m_odometry_noise)
      PVDEBUG("noisy speed baby!\n");
    
    PVDEBUG("ndof: %zu  qdot_len: %zu\n", ndof, *qdot_len);
    for (size_t ii(0); ii < *qdot_len; ++ii) {
      if (ii >= ndof) {
	for (/**/; ii < *qdot_len; ++ii)
	  qdot[ii] = 0;
	break;
      }
      if (m_odometry_noise) {
	PVDEBUG("  qdot[%zu] = noise(m_current_speed[%zu] = %g)\n",
		ii, ii, m_current_speed[ii]);
	qdot[ii] = (*m_odometry_noise)(m_current_speed[ii]);
      }
      else {
	PVDEBUG("  qdot[%zu] = m_current_speed[%zu] = %g\n",
		ii, ii, m_current_speed[ii]);
	qdot[ii] = m_current_speed[ii];
      }
    }
    
    *qdot_len = ndof;
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
    
    if(lidar->HaveNoiseModel())
      for(size_t is(0); is < *rho_len; ++is)
	rho[is] = lidar->GetNoisyRho(is);
    else
      for(size_t is(0); is < *rho_len; ++is)
	rho[is] = lidar->GetTrueRho(is);
    
    *t0 = lidar->GetT0();
    *t1 = lidar->GetT1();
    return 0;
  }
  
  
  void HAL::
  UpdateSpeeds()
  {
    memcpy(m_current_speed.get(), m_wanted_speed.get(), sizeof(double) * ndof);
  }
  
  
  void HAL::
  EnableOdometryNoise(const NoiseModel * noise)
  {
    m_odometry_noise = noise;
  }
  
  
  void HAL::
  DisableOdometryNoise()
  {
    m_odometry_noise = 0;
  }
  
}
