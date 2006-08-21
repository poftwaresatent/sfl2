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


#ifndef SUNFLOWER_ROBOTMODEL_HPP
#define SUNFLOWER_ROBOTMODEL_HPP


#include <sfl/util/numeric.hpp>
#include <sfl/util/Hull.hpp>
#include <boost/shared_ptr.hpp>
#include <vector>


namespace sfl {


  /**
     Encapsulates geometric, kinematic, and dynamic models of the
     robot. The geometry is represented as a hull (set of polygons),
     kinematics are hard coded for differential drive robots, and
     dynamics are simple bounds on speeds and accelerations.
  */
  class RobotModel
  {
  public:
    /**
       Abstract class for providing actual numerical values to
       RobotModel. It would be nice to retrieve this information from
       the HAL, which knows about most other such information anyways.
    */
    class Parameters{
    public:
      const double safetyDistance;
      const double wheelBase;
      const double wheelRadius;
      const double qdMax;
      const double qddMax;
      const double sdMax;
      const double thetadMax;
      const double sddMax;
      const double thetaddMax;
      
      Parameters(double safetyDistance,
		 double wheelBase,
		 double wheelRadius,
		 double qdMax,
		 double qddMax,
		 double sdMax,
		 double thetadMax,
		 /** suggest 0.75*wheelRadius*qddMax */
		 double sddMax,
		 /** suggest 1.5*wheelRadius*qddMax/wheelBase */
		 double thetaddMax);
    };
    
    
    /** \todo Good to pass parameters as instance? */
    RobotModel(Parameters parameters,
	       boost::shared_ptr<const Hull> hull);
    
    /**
       \return The robot's hull (without safety buffer zone).
    */
    boost::shared_ptr<const Hull> GetHull() const;
    
    /**
       \return The robot's hull with safety buffer zone
       (Parameters::safetyDistance).
    */
    boost::shared_ptr<const Hull> GetSafetyHull() const;    
    
    /**
       \return The safety distance.
       \todo should be renamed SafetyDistance().
    */
    double SecurityDistance() const;
    
    /**
       \return The wheel base (distance between the two drive wheels
       on a differetially driven robot).
    */
    double WheelBase() const;
    
    /**
       \return The radius of the drive wheels.
    */
    double WheelRadius() const;
    
    /**
       \return The maximum actuator speed [rad/s].
    */
    double QdMax() const;
    
    /**
       \return The maximum actuator acceleration [rad/s/s].
    */
    double QddMax() const;
    
    /**
       \return The maximum (global) forward speed [m/s].
    */
    double SdMax() const;
    
    /**
       \return The maximum (global) rotational speed [rad/s].
    */
    double ThetadMax() const;
    
    /**
       \return The maximum (global) acceleration [m/s/s].
    */
    double SddMax() const;
    
    /**
       \return The maximum (global) rotational acceleration [rad/s/s].
    */
    double ThetaddMax() const;
    
    /**
       Direct kinematic model for differential drive robots. Given
       left and right actuator speeds, returns the corresponding
       global translational and rotational speeds. Takes into account
       the special cases of pure translation and pure rotation.
    */
    void Actuator2Global(double qdl, double qdr,
			 double & sd, double & thetad) const;
    
    /**
       Indirect kinematic model for differential drive robots. Given
       the global translational and rotational speeds, calculates the
       corresponding left and right actuator speeds.
    */
    void Global2Actuator(double sd, double thetad,
			 double & qdl, double & qdr) const;
    
    /**
       Given current actuator wheel speeds, predicts the robot's
       position if it brakes along a constant-curvature path until
       standstill. An additional safety delay (before the braking
       maneuver starts) can be added as well. The prediction is
       expressed in the local frame of reference, that is to say the
       current robot pose.
       
       \note See PredictStandstillGlob() for a version that takes
       global speeds as input.
    */    
    void PredictStandstillAct(double qdl, double qdr, double safety_delay,
			      double & dx, double & dy, double & dtheta) const;
    
    /**
       Like PredictStandstillAct() but takes global wheel speeds.
    */    
    void PredictStandstillGlob(double sd, double thetad, double safety_delay,
			       double & dx, double & dy, double & dtheta)
      const;
    
    /**
       Given (global) translational and rotational speeds, predicts
       the robot's position at a given timestep in the future. The
       prediction is expressed in the local frame of reference, that
       is to say the current robot pose.
    */
    static void LocalKinematics(double sd, double thetad,
				double timestep,
				double & deltax,
				double & deltay,
				double & deltatheta);
    
    
  protected:
    const Parameters m_params;
    boost::shared_ptr<const Hull> m_hull;
    boost::shared_ptr<const Hull> m_safety_hull;
  };

}

#endif // SUNFLOWER_ROBOTMODEL_HPP
