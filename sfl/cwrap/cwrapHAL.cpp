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


#include "cwrapHAL.hpp"


namespace sfl_cwrap {
  
  
  cwrapHAL::
  cwrapHAL(struct cwrap_hal_s * hal)
    : m_hal(hal)
  {
  }
  
  
  int cwrapHAL::
  time_get(struct ::timespec * stamp)
  { return m_hal->time_get(stamp); }
  
  
  int cwrapHAL::
  odometry_set(double x, double y, double theta,
	       double sxx, double syy, double stt,
	       double sxy, double sxt, double syt)
  { return m_hal->odometry_set(x, y, theta,
			       sxx, syy, stt,
			       sxy, sxt, syt); }
  
  
  int cwrapHAL::
  odometry_get(struct ::timespec * stamp,
	       double * x, double * y, double * theta,
	       double * sxx, double * syy, double * stt,
	       double * sxy, double * sxt, double * syt)
  { return m_hal->odometry_get(stamp, x, y, theta,
			       sxx, syy, stt,
			       sxy, sxt, syt); }
  
  
  int cwrapHAL::
  speed_set(double qdl, double qdr)
  { return m_hal->speed_set(qdl, qdr); }
  
  
  int cwrapHAL::
  scan_get(int channel, double * rho, int rho_len,
	   struct ::timespec * t0, struct ::timespec * t1)
  { return m_hal->scan_get(channel, rho, rho_len, t0, t1); }
  
}
