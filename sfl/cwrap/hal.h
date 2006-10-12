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


#ifndef CWRAP_HAL_H
#define CWRAP_HAL_H

#ifdef __cplusplus
extern "C" {
#endif // __cplusplus
  

#include <stddef.h>

  
  /**
     Collection of HAL function pointers.
     
     Example usage:
     \code
     struct cwrap_hal_s *hal_cb = calloc(1, sizeof(*hal_cb));
     if(0 == hal_cb){
       fprintf(stderr, "ERROR: out of memory\n");
       exit(EXIT_FAILURE);
       }
       hal_cb->time_get     = time_get;
       hal_cb->odometry_set = odometry_set;
       hal_cb->speed_set    = sflSetSpeed;
       hal_cb->odometry_get = odometry_get_generic;
       hal_cb->scan_get     = scan_get_generic;
     \endcode
  */
  struct cwrap_hal_s {
    int (*time_get)(struct cwrap_hal_s * self, struct timespec * stamp);
    int (*odometry_set)(struct cwrap_hal_s * self,
			double x, double y, double theta,
			double sxx, double syy, double stt,
			double sxy, double sxt, double syt);
    int (*odometry_get)(struct cwrap_hal_s * self, struct timespec * stamp,
			double * x, double * y, double * theta,
			double * sxx, double * syy, double * stt,
			double * sxy, double * sxt, double * syt);
    int (*speed_set)(struct cwrap_hal_s * self, double qdl, double qdr);
    int (*speed_get)(struct cwrap_hal_s * self, double * qdl, double * qdr);
    
    /**
       \param channel (in): scanner channel number
       \param rho_len (in): length of the rho array
       
       \param rho (out): array of distances, unit = [m]
       \param t0 (out): lower bound on acquisition time
       \param t1 (out): upper bound on acquisition time

       \return 0 on success
    */
    int (*scan_get)(struct cwrap_hal_s * self,
		    int channel, double * rho, size_t * rho_len,
		    struct timespec * t0, struct timespec * t1);
    
    /**
       \param channel: scanner channel number
       
       \return 0 on success
    */
    int (*scan_init)(struct cwrap_hal_s * self, int channel);
    
    /**
       \param channel: scanner channel number
       
       \return 0 on success
    */
    int (*scan_end)(struct cwrap_hal_s * self, int channel);
  };
  
  
#ifdef __cplusplus
} // extern "C"
#endif // __cplusplus

#endif // CWRAP_HAL_H
