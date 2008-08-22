/*
 * Copyright (c) 2008 Roland Philippsen <roland DOT philippsen AT gmx DOT net>
 *
 * BSD license:
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holder nor the names of
 *    contributors to this software may be used to endorse or promote
 *    products derived from this software without specific prior written
 *    permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHORS AND CONTRIBUTORS ``AS IS''
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED
 * TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
 * PARTICULAR PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT
 * HOLDER OR THE CONTRIBUTORS TO THIS SOFTWARE BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef ROSHAL_HPP
#define ROSHAL_HPP

#include <sfl/api/HAL.hpp>
#include <vector>

namespace {
  class ROSHALNode;
}


class ROSHAL
  : public sfl::HAL
{
public:
  ROSHAL(size_t nscanners, std::ostream * dbgos);
  virtual ~ROSHAL();
  
  virtual int time_get(sfl::timespec_t * stamp);
  
  virtual int odometry_set(double x, double y, double theta,
			   double sxx, double syy, double stt,
			   double sxy, double sxt, double syt);
  
  virtual int odometry_get(sfl::timespec_t * stamp,
			   double * x, double * y, double * theta,
			   double * sxx, double * syy, double * stt,
			   double * sxy, double * sxt, double * syt);
  
  virtual int speed_set(const double * qdot,
			size_t * qdot_len);
  
  virtual int speed_get(double * qdot,
			size_t * qdot_len);
  
  virtual int scan_get(int channel, double * rho,
		       size_t * rho_len,
		       sfl::timespec_t * t0, sfl::timespec_t * t1);
  
protected:
  ROSHALNode * m_node;
  std::ostream * m_dbgos;  
};

#endif // ROSBOT_HPP
