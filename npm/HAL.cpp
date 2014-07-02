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
#include <strings.h>
#include <string.h>


using namespace sfl;
using namespace boost;


namespace npm {
  
  
  HAL::
  HAL(RobotServer * owner)
    : m_ndof(0),
      m_owner(owner),
      m_wanted_speed(0),
      m_current_speed(0),
      m_odometry_noise(0)
  {
  }
  
  
  HAL::
  ~HAL()
  {
    free(m_wanted_speed);
    free(m_current_speed);
  }
  
  
  int HAL::
  speed_set(const double * qdot, size_t * qdot_len)
  {
    if (( ! qdot) || ( ! qdot_len)) {
      PDEBUG("qdot (%p) and qdot_len (%p) must not be null\n", qdot, qdot_len);
      return -1;
    }
    
    if (*qdot_len > m_ndof) {
      double *tmp1, *tmp2;
      tmp1 = (double*) realloc(m_wanted_speed, *qdot_len * sizeof *tmp1);
      if ( !tmp1) {
	PDEBUG("out of memory");
	return -2;
      }
      tmp2 = (double*) realloc(m_current_speed, *qdot_len * sizeof *tmp2);
      if ( !tmp2) {
	PDEBUG("out of memory");
	return -2;
      }
      m_wanted_speed = tmp1;
      m_current_speed = tmp2;
      m_ndof = *qdot_len;
    }
    
    PVDEBUG("ndof: %zu  qdot_len: %zu\n", ndof, *qdot_len);
    for (size_t ii(0); ii < m_ndof; ++ii) {
      if (ii >= *qdot_len) {
	for (/**/; ii < m_ndof; ++ii)
	  m_wanted_speed[ii] = 0;
	break;
      }
      PVDEBUG("  m_wanted_speed[%zu] = qdot[%zu] = %g\n", ii, ii, qdot[ii]);
      m_wanted_speed[ii] = qdot[ii];
    }
    *qdot_len = m_ndof;
    return 0;
  }
  
  
  int HAL::
  speed_get(double * qdot, size_t * qdot_len)
  {
    if (*qdot_len > m_ndof) {
      double *tmp1, *tmp2;
      tmp1 = (double*) realloc(m_wanted_speed, *qdot_len * sizeof *tmp1);
      if ( !tmp1) {
	PDEBUG("out of memory");
	return -2;
      }
      tmp2 = (double*) realloc(m_current_speed, *qdot_len * sizeof *tmp2);
      if ( !tmp2) {
	PDEBUG("out of memory");
	return -2;
      }
      m_wanted_speed = tmp1;
      m_current_speed = tmp2;
      for (size_t ii(m_ndof); ii < *qdot_len; ++ii) {
	tmp1[ii] = 0.0;
	tmp2[ii] = 0.0;
      }
      m_ndof = *qdot_len;
    }
    
    PVDEBUG("ndof: %zu  qdot_len: %zu\n", m_ndof, *qdot_len);
    for (size_t ii(0); ii < *qdot_len; ++ii) {
      if (ii >= m_ndof) {
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
    
    *qdot_len = m_ndof;
    return 0;
  }
  
  
  void HAL::
  UpdateSpeeds()
  {
    memcpy(m_current_speed, m_wanted_speed, sizeof(double) * m_ndof);
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
