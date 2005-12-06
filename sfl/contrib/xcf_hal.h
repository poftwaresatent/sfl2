/*
 * Copyright (c) 2005 Universitaet Bielefeld, Germany
 *                    and LAAS/CNRS, France
 *
 * Authors: Thorsten Spexard <tspexard at techfak dot uni-bielefeld dot de>
 *          Roland Philippsen <roland dot philippsen at gmx dot net>
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


#ifndef XCF_HAL_H
#define XCF_HAL_H

#ifdef __cplusplus
extern "C" {
#endif // __cplusplus
  
  
#include <stdint.h>
#include <stdio.h>
  
  
  void xcf_hal_dryrun_on(FILE * stream);
  
  void xcf_hal_dryrun_off();
  
  
  
  int xcf_odometry_init();
  
  int xcf_odometry_get(double * x, double * y, double * theta_rad,
		       uint64_t * timestamp_ms);
  
  int xcf_odometry_end();
  
  
  
  int xcf_speed_init();
  
  int xcf_speed_set(double v, double w);
  
  int xcf_speed_end();
  
  
  
  int xcf_scan_init(int channel);
  
  int xcf_scan_get(int channel, double * rho, int rho_len,
		   uint64_t * timestamp_ms);
  
  int xcf_scan_end(int channel);
  
  
  //INPUT from ESV
  int xcf_navgoal_init();
  
  /** \return <ul><li>  0: success, new location received </li>
                  <li>  1: no new location (empty stream) </li>
                  <li> <0: ERROR                          </li></ul> */
  int xcf_navgoal_receive(char * location, size_t location_len,
			  /** to be given back through xcf_navresult_send() */
			  int * transaction_id,
			  /** to be given back through xcf_navresult_send() */
			  char * esv_state, size_t esv_state_len);
  int xcf_navgoal_end();
  
  
  //OUTPUT to ESV
  int xcf_navresult_init();

  int xcf_navresult_send(const char * result,
			 /** from xcf_navgoal_receive() */
			 int transaction_id,
			 /** from xcf_navgoal_receive() */
			 const char * esv_state);

  int xcf_navresult_end();
  
#ifdef __cplusplus
}
#endif // __cplusplus

#endif // XCF_HAL_H
