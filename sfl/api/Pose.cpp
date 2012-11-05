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


#include "Pose.hpp"


namespace sfl {


  Pose::
  Pose()
    : Frame(),
      m_sxx(1), m_syy(1), m_stt(1),
      m_sxy(0), m_sxt(0), m_syt(0)
  {
  }


  Pose::
  Pose(const Frame & frame)
    : Frame(frame),
      m_sxx(1), m_syy(1), m_stt(1),
      m_sxy(0), m_sxt(0), m_syt(0)
  {
  }
  
  
  Pose::
  Pose(double x, double y, double theta)
    : Frame(x, y, theta),
      m_sxx(1), m_syy(1), m_stt(1),
      m_sxy(0), m_sxt(0), m_syt(0)
  {
  }
  
  
  Pose::
  Pose(const Frame & frame,
       double sxx, double syy, double stt,
       double sxy, double sxt, double syt)
    : Frame(frame),
      m_sxx(sxx), m_syy(syy), m_stt(stt),
      m_sxy(sxy), m_sxt(sxt), m_syt(syt)
  {
  }
  
  
  Pose::
  Pose(double x, double y, double theta,
       double sxx, double syy, double stt,
       double sxy, double sxt, double syt)
    : Frame(x, y, theta),
      m_sxx(sxx), m_syy(syy), m_stt(stt),
      m_sxy(sxy), m_sxt(sxt), m_syt(syt)
  {
  }
  
  
  double Pose::
  Sxx() const
  {
    return m_sxx;
  }
  
  
  double Pose::
  Syy() const
  {
    return m_syy;
  }
  
  
  double Pose::
  Stt() const
  {
    return m_stt;
  }
  
  
  double Pose::
  Sxy() const
  {
    return m_sxy;
  }
  
  
  double Pose::
  Sxt() const
  {
    return m_sxt;
  }
  
  
  double Pose::
  Syt() const
  {
    return m_syt;
  }
  
  
  void Pose::
  SetVar(double sxx, double syy, double stt,
	 double sxy, double sxt, double syt)
  {
    m_sxx = sxx;
    m_syy = syy;
    m_stt = stt;
    m_sxy = sxy;
    m_sxt = sxt;
    m_syt = syt;
  }
  
  
  Pose const & Pose::
  operator = (Frame const &rhs)
  {
    Set(rhs);
    m_sxx = 1.;
    m_syy = 1.;
    m_stt = 1.;
    m_sxy = 0.;
    m_sxt = 0.;
    m_syt = 0.;
    return *this;
  }

}
