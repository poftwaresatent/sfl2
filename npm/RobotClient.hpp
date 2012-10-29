/* 
 * Copyright (C) 2006 Roland Philippsen <roland dot philippsen at gmx dot net>
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


#ifndef NPM_ROBOTCLIENT_HPP
#define NPM_ROBOTCLIENT_HPP

#include <fpplib/configurable.hpp>
#include <boost/shared_ptr.hpp>


namespace sfl {
  class Line;
  class Polygon;
  class Goal;
  class Frame;
  class Scanner;
}


namespace npm {
  
  class HAL;
  class Lidar;
  class Sharp;
  class Drawing;
  class Camera;
  class DiffDrive;
  class HoloDrive;
  class BicycleDrive;
  class RobotServer;
  
  
  /**
     Base class for implementing robots.
  */
  class RobotClient
    : public fpplib::Configurable
  {
  public:
    RobotClient(std::string const &name);
    
    virtual bool Initialize(RobotServer &server);
    
    /** Entry point for simulating the robot. It should calculate the
	next action based on sensor readings (which are updated by the
	simulator) and should end by assigning new motor commands to the
	actuators.
	
	\return true if the robot is in a runnable state. If you
	return false, the simulator will switch to step-by-step
	mode. It is up to subclasses to print appropriate error
	message or try to recover.
    */
    virtual bool PrepareAction(double timestep) = 0;
    
    /** Hook for initially placing the robot, empty default implementation. */
    virtual void InitPose(double x, double y, double theta) {}
    
    /** Hook for placing the robot during simulation, empty default
	implementation. */
    virtual void SetPose(double x, double y, double theta) {}
    
    /** Hook for knowing where the robot thinks it is, default
	implementation uses the true pose, which should in principle
	not be available to subclasses. */
    virtual bool GetPose(double & x, double & y, double & theta) { return false; }
    
    /** Hook to set the robot's goal, empty default implementation. */
    virtual void SetGoal(double timestep, const sfl::Goal & goal) {}
    
    /** Hook to query the robot's current goal, default implementation
	returns an 'invalid' pointer. */
    virtual boost::shared_ptr<const sfl::Goal> GetGoal();
    
    /** Hook for knowing when the robot thinks it has reached its
	goal, default implementation always returns false. */
    virtual bool GoalReached() { return false; }
    
  protected:
    boost::shared_ptr<HAL> m_hal; // set via Initialize
    
  private:
    friend class RobotServer;
    
    bool m_enable_trajectory;
    
    bool m_noisy_odometry;
    double m_odometry_noise_min_factor;//(0.95); // factors <0 are ignored
    double m_odometry_noise_max_factor;//(1.05);
    double m_odometry_noise_min_offset;//(1); // if min>max then offsets
    double m_odometry_noise_max_offset;//(-1); // are ignored
    
    bool m_noisy_scanners;
    double m_scanner_noise_min_factor;//(-1); // factors <0 are ignored
    double m_scanner_noise_max_factor;//(-1);
    double m_scanner_noise_min_offset;//(-0.1); // if min>max then offsets
    double m_scanner_noise_max_offset;//( 0.1); // are ignored
    
    double m_camera_zoom;//(2);
  };
  
}

#endif // NPM_ROBOTCLIENT_HPP
