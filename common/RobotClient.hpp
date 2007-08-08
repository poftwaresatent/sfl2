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


#include <boost/shared_ptr.hpp>


namespace sfl {
  class Line;
  class Goal;
  class Frame;
  class Scanner;
}


namespace npm {
  
  class HAL;
  class HALFactory;
  class RobotDescriptor;
  class Lidar;
  class Sharp;
  class Drawing;
  class Camera;
  class DiffDrive;
  class HoloDrive;
  class BicycleDrive;
  class RobotServer;
  class World;
  
  
  /**
     Pure abstract interface. Allows subclasses of Robot to "construct"
     their hardware and register drawings and such.
  */
  class RobotClient
  {
  public:
    RobotClient(boost::shared_ptr<RobotDescriptor> descriptor,
		const World & world, bool enable_trajectory);
    
    RobotClient(const HALFactory & hal_factory,
		boost::shared_ptr<RobotDescriptor> descriptor,
		const World & world, bool enable_trajectory);
    
    virtual ~RobotClient();
    
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
    virtual void GetPose(double & x, double & y, double & theta);
    
    /** Hook to set the robot's goal, empty default implementation. */
    virtual void SetGoal(double timestep, const sfl::Goal & goal) = 0;//{}
    
    /** Hook to query the robot's current goal, default implementation
	returns an 'invalid' pointer. */
    virtual boost::shared_ptr<const sfl::Goal> GetGoal();
    
    /** Hook for knowing when the robot thinks it has reached its
	goal, default implementation always returns false. */
    virtual bool GoalReached() { return false; }
    
    /** npm::HAL subclasses sfl::HAL to easily interface libsunflower. */
    boost::shared_ptr<HAL> GetHAL();
    
    /** This allows you to retrieve runtime options, a bit like
	environment variables. */
    boost::shared_ptr<RobotDescriptor> GetDescriptor();
    
    /** using this means you're cheating... */
    boost::shared_ptr<RobotServer> GetServer() { return m_server; }
    
    /** using this means you're cheating... */
    boost::shared_ptr<const RobotServer> GetServer() const { return m_server; }
    
  protected:
    /** Add a line to the robot's body. */
    void AddLine(const sfl::Line & line);
    
    /** Add a Drawing subclass to the robot. */
    void AddDrawing(boost::shared_ptr<Drawing> drawing);
    
    /** Convenient shortcut if you want to transfer ownership anyways. */
    void AddDrawing(Drawing * drawing);
    
    /** Add a Camera subclass to the robot. */
    void AddCamera(boost::shared_ptr<Camera> camera);
    
    /** Convenient shortcut if you want to transfer ownership anyways. */
    void AddCamera(Camera * camera);
    
    /** Factory method for robot subclasses to create their sensors. */
    boost::shared_ptr<Lidar>
    DefineLidar(const sfl::Frame & mount, size_t nscans, double rhomax,
		double phi0, double phirange, int hal_channel);
    
    /** Variant for clients who created their own scanner... */
    boost::shared_ptr<Lidar>
    DefineLidar(boost::shared_ptr<sfl::Scanner> scanner);

    /** See DefineLidar(). */
    boost::shared_ptr<Sharp>
    DefineSharp(const sfl::Frame & mount, double rmax, int channel);
    
    /** Factory method for creating a differential drive actuator. */
    boost::shared_ptr<DiffDrive>
    DefineDiffDrive(double wheelbase, double wheelradius);
    
    /** Factory method for creating a holonomic drive actuator. */
    boost::shared_ptr<HoloDrive>
    DefineHoloDrive(double axislength);

    /** Factory method for creating a bicycle drive actuator. */
    boost::shared_ptr<BicycleDrive>
    DefineBicycleDrive(double wheelbase, double wheelradius, double axlewidth);
    
      //  private:
  public:
    friend class Simulator;
    boost::shared_ptr<RobotServer> m_server;
  };
  
}

#endif // NPM_ROBOTCLIENT_HPP
