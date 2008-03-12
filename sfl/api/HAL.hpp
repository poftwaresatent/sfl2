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


#ifndef SUNFLOWER_HAL_HPP
#define SUNFLOWER_HAL_HPP


#include <cstddef>

#ifndef WIN32
/** Declared in <time.h> but don't use system clock: The HAL is used
    as central "wall clock" so that simulation can freeze time when
    needed. */
struct timespec;
#else
# include <sfl/util/win32.hpp>
#endif // WIN32


namespace sfl {

	
#ifdef WIN32
	typedef fake_timespec timespec_t;
#else // WIN32
	typedef struct ::timespec timespec_t;
#endif // WIN32

  
  /**
     This class serves as proxy for the hardware abstraction
     layer. This additional indirection makes it possible to keep the
     object-oriented parts of sunflower sort of independent from the
     HAL implementation. For example, it makes it easy to switch
     between Fred's HAL of Robox and Smartease, using GenoM, or
     working with the nepumuk simulator - all with one and the same
     sunflower code.

     \note If you have an existing hardware abstraction layer coded in
     C, have a look at sfl_cwrap::cwrapHAL in the cwrap/ directory. It
     takes a collection of function pointers in a struct cwrap_hal_s
     as ctor argument, which simplifies interfacing your C
     implementation with libsunflower.
  */
  
  class HAL
  {
  public:
    virtual ~HAL() { }
    
    /** \return 0 on success. */
    virtual int time_get(timespec_t * stamp) = 0;
    
    /** \return 0 on success. */
    virtual int odometry_set(double x, double y, double theta,
			     double sxx, double syy, double stt,
			     double sxy, double sxt, double syt) = 0;
    
    /** \return 0 on success. */
    virtual int odometry_get(timespec_t * stamp,
			     double * x, double * y, double * theta,
			     double * sxx, double * syy, double * stt,
			     double * sxy, double * sxt, double * syt) = 0;
    
    
    /**
       Copy joint speeds from a user-supplied array 'qdot' which has
       'qdot_len' elements. If qdot_len is smaller than the number of
       joints, set the remaining joint velocities to zero. If qdot_len
       is larger than the number of joints, discard the extra
       data. After copying, qdot_len is set to the number of degrees
       of freedom.
       
       \return 0 on success.
    */
    virtual int speed_set(const double * qdot,
			  /** IN: size of qdot[], OUT: number of joints */
			  size_t * qdot_len) = 0;
    
    /**
       Copy joint speeds into a user-supplied array 'qdot' which has
       'qdot_len' elements. If qdot_len is smaller than the number of
       joints, discard the extra data. If qdot_len is larger than the
       number of joints, fill the extra elements with zeros. After
       copying, qdot_len is set to the number of degrees of freedom.
       
       \return 0 on success.
    */
    virtual int speed_get(double * qdot,
			  /** IN: size of qdot[], OUT: number of joints */
			  size_t * qdot_len) = 0;
    
    /** \note rho_len is input AND output: If there are fewer scan
	points than (in) rho_len available, this is reflected by the
	(out) value of rho_len. rho[ii] at ii >= (in) rho_len ARE NOT
	UPDATED, it is up to the caller to do something sensible such
	as setting them to max range or ignoring them. If there are
	MORE than (in) rho_len data points, the scan data is simply
	truncated.
	\return 0 on success. */
    virtual int scan_get(int channel, double * rho,
			 /** IN: size of rho[], OUT: scan length */
			 size_t * rho_len,
			 timespec_t * t0, timespec_t * t1) = 0;
    
    
    int deprecated_speed_set(double qdl, double qdr) {
      double qd[2] = { qdl, qdr };
      size_t len(2);
      return speed_set(qd, &len);
    }
    
    
    int deprecated_speed_get(double * qdl, double * qdr) {
      double qd[2];
      size_t len(2);
      int const stat(speed_get(qd, &len));
      if (0 != stat)
	return stat;
      *qdl = qd[0];
      *qdr = qd[1];
      return 0;
    }
    
  };
  
}

#endif // SUNFLOWER_HAL_HPP
