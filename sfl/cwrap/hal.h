/*
 * Copyright (c) 2005 CNRS/LAAS
 *
 * Author: Roland Philippsen <roland.philippsen@gmx.net>
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


#ifndef CWRAP_HAL_H
#define CWRAP_HAL_H

#ifdef __cplusplus
extern "C" {
#endif // __cplusplus
  
  
  struct cwrap_hal_s {
    int (*time_get)(struct timespec * stamp);
    int (*odometry_set)(double x, double y, double theta,
			double sxx, double syy, double stt,
			double sxy, double sxt, double syt);
    int (*odometry_get)(struct timespec * stamp,
			double * x, double * y, double * theta,
			double * sxx, double * syy, double * stt,
			double * sxy, double * sxt, double * syt);
    int (*speed_set)(double qdl, double qdr);
    int (*scan_get)(int channel, double * rho, int rho_len,
		    struct timespec * t0, struct timespec * t1);
  };
  
  
#ifdef __cplusplus
} // extern "C"
#endif // __cplusplus

#endif // CWRAP_HAL_H
