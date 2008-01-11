/*
 * Copyright (c) 2005 CNRS/LAAS
 *
 * Author: Roland Philippsen <roland dot philippsen at gmx dot net>
 *
 * Permission to use, copy, modify, and distribute this software for any
 * purpose with or without fee is hereby granted, provided that the above
 * copyright notice and this permission notice appear in all copies.
 *
 * THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
 * WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
 * ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
 * WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
 * ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
 * OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
 */


#ifndef CWRAP_HAL_HPP
#define CWRAP_HAL_HPP


#include <sfl/api/HAL.hpp>
#include <sfl/cwrap/hal.h>


namespace sfl_cwrap {
  
  
  /** Forwards all calls to a C struct of function pointers that is
      provided at construction time. */
  class cwrapHAL
    : public sfl::HAL
  {
  public:
    cwrapHAL(struct cwrap_hal_s * hal);
    
    virtual int time_get(struct ::timespec * stamp);
    virtual int odometry_set(double x, double y, double theta,
			     double sxx, double syy, double stt,
			     double sxy, double sxt, double syt);
    virtual int odometry_get(struct ::timespec * stamp,
			     double * x, double * y, double * theta,
			     double * sxx, double * syy, double * stt,
			     double * sxy, double * sxt, double * syt);
    virtual int speed_set(const double * qdot, size_t * qdot_len);
    virtual int speed_get(double * qdot, size_t * qdot_len);
    virtual int scan_get(int channel, double * rho, size_t * rho_len,
			 struct ::timespec * t0, struct ::timespec * t1);
    
  private:
    struct cwrap_hal_s * m_hal;
  };

}

#endif // CWRAP_HAL_HPP
