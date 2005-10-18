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
      const double securityDistance;
      const double wheelBase;
      const double wheelRadius;
      const double qdMax;
      const double qddMax;
      const double sdMax;
      const double thetadMax;
      const double sddMax;
      const double thetaddMax;
      
      Parameters(double securityDistance,
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
    RobotModel(double timestep,
	       Parameters parameters,
	       boost::shared_ptr<const Hull> hull);
    
    
    /**
       \return The timestep.
    */
    double Timestep() const;
    
    /**
       \return A reference to the robot's hull.
    */
    boost::shared_ptr<const Hull> GetHull() const;
    
    /**
       \return The security distance.
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
  
    /**
       Just like the LocalKinematics() with timestep parameter, but
       the timestep parameter defaults to Timestep().
       
       \todo Maybe refactor to one method with the timestep as
       parameter that defaults to Timestep()?
    */
    void LocalKinematics(double sd, double thetad,
			 double & deltax,
			 double & deltay,
			 double & deltatheta) const;
        
    
  protected:
    const double m_timestep;
    const Parameters m_params;
    boost::shared_ptr<const Hull> m_hull;
  };

}

#endif // SUNFLOWER_ROBOTMODEL_HPP
