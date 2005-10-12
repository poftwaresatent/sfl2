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


#ifndef SUNFLOWER_HAL_PROXY_HPP
#define SUNFLOWER_HAL_PROXY_HPP


namespace sfl {

  
  /**
     This class serves as proxy for the hardware abstraction
     layer. This additional indirection makes it possible to keep the
     object-oriented parts of sunflower highly independent from the
     HAL implementation. For example, it makes it easy to switch
     between Fred's HAL of Robox and Smartease, using GenoM, or
     working with the nepumuk simulator - all with one and the same
     sunflower code.

     \note This class is work in progress depending on the need for
     sunflower-light and nepumuk-sfl-light development.
  */

  class HALProxy
  {
  public:
    virtual ~HALProxy() { }
    
    /** \return 0 on success. */
    virtual int hal_time_get(struct timespec & stamp) = 0;
    
    /** \return 0 on success. */
    virtual int hal_odometry_set(double x, double y, double theta,
				 double sxx, double syy, double stt,
				 double sxy, double sxt, double syt) = 0;
    
    /** \return 0 on success. */
    virtual int hal_odometry_get(struct timespec & stamp,
				 double & x, double & y, double & theta,
				 double & sxx, double & syy, double & stt,
				 double & sxy, double & sxt, double & syt) = 0;
    
    /** \return 0 on success. */
    virtual int hal_speed_set(double qdl, double qdr) = 0;
  };

}

#endif // SUNFLOWER_HAL_PROXY_HPP
