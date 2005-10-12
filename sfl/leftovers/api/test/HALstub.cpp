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


#include "HALstub.hpp"
using namespace std;

extern "C" {
#include <sfl/hal/include/hal.h>
}


static int __error_code = 0;
static uint16_t __nranges = 361;
static double __speed_l = 0;
static double __speed_r = 0;
static double __x = 0;
static double __y = 0;
static double __theta = 0;
static struct timespec __last_stamp;


namespace sunflower
{

  void HALstub::
  EmulateError(int error_code)
  {
    __error_code = error_code;
  }


  void HALstub::
  SetNranges(unsigned int nranges)
  {
    __nranges = nranges;
  }


  void HALstub::
  SetPose(const Pose & pose)
  {
    __x = pose.X();
    __y = pose.Y();
    __theta = pose.Theta();
  }


  Pose HALstub::
  GetPose()
  {
    return Pose(__x, __y, __theta);
  }


  Timestamp HALstub::
  GetLastTimestamp()
  {
    return Timestamp(__last_stamp);
  }

}


//////////////////////////////////////////////////
// The real fake starts here.


int hal_laser_get_channel(uint8_t channel,
			  struct hal_laser_data_s * data)
{
  if(__error_code != 0)
    return __error_code;

  hal_time_get( & data->timestamp);
  for(uint16_t i(0); i < data->nranges; ++i)
    data->ranges[i] = i;
  
  return 0;
}


int hal_laser_get_nranges(uint8_t channel,
			  uint16_t *nranges)
{
  if(__error_code != 0)
    return __error_code;

  * nranges = __nranges;

  return 0;  
}


int hal_speed_get(struct hal_speed_data_s *data)
{
  if(__error_code != 0)
    return __error_code;

  hal_time_get( & data->timestamp);
  data->speed_l = __speed_l;
  data->speed_r = __speed_r;

  return 0;
}


int hal_speed_set(struct hal_speed_data_s data)
{
  if(__error_code != 0)
    return __error_code;

  __speed_l = data.speed_l;
  __speed_r = data.speed_r;

  return 0;
}


int hal_odometry_get(struct hal_odometry_data_s *data)
{
  if(__error_code != 0)
    return __error_code;

  hal_time_get( & data->timestamp);
  data->x = __x;
  data->y = __y;
  data->theta = __theta;
  data->cov_matrix[0][0] = 1;
  data->cov_matrix[1][1] = 1;
  data->cov_matrix[2][2] = 1;
  data->cov_matrix[0][1] = 0;
  data->cov_matrix[0][2] = 0;
  data->cov_matrix[1][0] = 0;
  data->cov_matrix[1][2] = 0;
  data->cov_matrix[2][0] = 0;
  data->cov_matrix[2][1] = 0;

  return 0;
}


int hal_odometry_set(struct hal_odometry_data_s data)
{
  if(__error_code != 0)
    return __error_code;

  __x = data.x;
  __y = data.y;
  __theta = data.theta;

  return 0;
}


int hal_time_get(struct timespec *time)
{
  if(__error_code != 0)
    return __error_code;
  
  struct timeval val;
  if(gettimeofday( & val, NULL) != 0){
    cerr << "HALstub: hal_time_get(): gettimeofday() error: "
	 << strerror(errno);
    abort();
  }
  TIMEVAL_TO_TIMESPEC( & val, time);

  __last_stamp = * time;;

  return 0;
}
