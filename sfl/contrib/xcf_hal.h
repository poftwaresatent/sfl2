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
  
  
  /** Turns on messages to stream instead of actually doing
      anything. Set stream to NULL, or use xcf_hal_dryrun_off(), to
      get back to work. */
  void xcf_hal_dryrun_on(FILE * stream);
  
  /** see xcf_hal_dryrun_on() */
  void xcf_hal_dryrun_off();
  
  /** \return description of last error */
  const char * xcf_hal_geterror();
  
  /** \return number of milliseconds since 1.1.1970 or something like that */
  uint64_t xcf_hal_timestamp();
  
  /** \return 0 on success, -1 if error (see xcf_hal_geterror()) */
  int xcf_odometry_init();
  
  /** \return 0 on success, -1 if error (see xcf_hal_geterror()) */
  int xcf_speed_init();
  
  /** \return 0 on success, -1 if error (see xcf_hal_geterror()) */
  int xcf_scan_init();
  
  /** \return 0 on success, -1 if error (see xcf_hal_geterror()) */
  int xcf_navgoal_init();
  
  /** \return 0 on success, -1 if error (see xcf_hal_geterror()) */
  int xcf_navresult_init();
  
  void xcf_odometry_end();
  void xcf_speed_end();
  void xcf_scan_end();
  void xcf_navgoal_end();
  void xcf_navresult_end();
  
  /** \return >=0 on success (0: new data, 1 ok but empty stream), -1
      if error (see xcf_hal_geterror()) */
  int xcf_odometry_receive(double * x, double * y, double * theta_rad,
			   uint64_t * timestamp_ms);
  
  /** \return >=0 on success (0: new data, 1 ok but empty stream), -1
      if error (see xcf_hal_geterror()) */
  int xcf_scan_receive(/** pointer to array to be filled with scan */
		       double * rho,
		       /** in: length of rho, out: number of entries */
		       int * rho_len,
		       /** timestamp of the data */
		       uint64_t * timestamp_ms);
  
  /** \return >=0 on success (0: new data, 1 ok but empty stream), -1
      if error (see xcf_hal_geterror()) */
  int xcf_navgoal_receive(char * location, size_t location_len,
			  /** to be given back through xcf_navresult_send() */
			  int * transaction_id,
			  /** to be given back through xcf_navresult_send() */
			  char * esv_state, size_t esv_state_len);
  
  /** \return 0 on success, -1 if error (see xcf_hal_geterror()) */
  int xcf_speed_send(double v, double w);
  
  /** \return 0 on success, -1 if error (see xcf_hal_geterror()) */
  int xcf_navresult_send(const char * result,
			 /** from xcf_navgoal_receive() */
			 int transaction_id,
			 /** from xcf_navgoal_receive() */
			 const char * esv_state);
  
#ifdef __cplusplus
}
#endif // __cplusplus

#endif // XCF_HAL_H
