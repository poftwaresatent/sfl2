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


#ifndef SUNFLOWER_POSE_HPP
#define SUNFLOWER_POSE_HPP


#include <sfl/util/Frame.hpp>


namespace sfl {


  /**
     Like Frame, but with covariance information.
  */
  class Pose
    : public Frame
  {
  public:
    /** Default position = (0, 0, 0), covariance = identity */
    Pose();

    /** Default covariance = identity */
    explicit Pose(const Frame & frame);

    /** Default covariance = identity */
    Pose(double x, double y, double theta);

    /** Construct from existing Frame instance, adding the provided
	covariances. */
    Pose(const Frame & frame,
	 double sxx, double syy, double stt,
	 double sxy, double sxt, double syt);

    /** Construct a Pose instance at (x, y, theta), with the provided
	covariances. */
    Pose(double x, double y, double theta,
	 double sxx, double syy, double stt,
	 double sxy, double sxt, double syt);
    
    /** Doesn't touch the position, only the covariance. */
    void SetVar(double sxx, double syy, double stt,
		double sxy, double sxt, double syt);
    
    /** \return The variance along the x-axis. */
    double Sxx() const;
    
    /** \return The variance along the y-axis. */
    double Syy() const;
    
    /** \return The variance along the theta-axis. */
    double Stt() const;

    /** \return The covariance between the x- and the y-axes. */
    double Sxy() const;

    /** \return The covariance between the x- and the theta-axes. */
    double Sxt() const;
    
    /** \return The covariance between the y- and the theta-axes. */
    double Syt() const;
    
  protected:
    double m_sxx, m_syy, m_stt, m_sxy, m_sxt, m_syt;
  };
  
}

#endif // SUNFLOWER_POSE_HPP
