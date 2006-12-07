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


#ifndef NPM_ROBOTSERVER_HPP
#define NPM_ROBOTSERVER_HPP


#include <boost/shared_ptr.hpp>
#include <vector>
#include <map>


namespace sfl {
  class Line;
  class Frame;
  class Scanner;
}


namespace npm {
  
  class Sensor;
  class Object;
  class World;
  class Lidar;
  class Sharp;
  class RobotDescriptor;
  class HAL;
  class HALFactory;
  class Drawing;
  class Camera;
  class Drive;
  class DiffDrive;
  class HoloDrive;
  class BicycleDrive;
  
  
  class RobotServer
  {
  private:
    RobotServer(const RobotServer &);
    
  public:
    typedef std::vector<boost::shared_ptr<const sfl::Frame> > trajectory_t;
    
    RobotServer(boost::shared_ptr<RobotDescriptor> descriptor,
		const World & world, bool enable_trajectory);
    
    RobotServer(const HALFactory & hal_factory,
		boost::shared_ptr<RobotDescriptor> descriptor,
		const World & world, bool enable_trajectory);
    
    void UpdateAllSensors();
    void UpdateSensor(Sensor & sensor) const;
    void SimulateAction(double timestep);
    void InitializeTruePose(const sfl::Frame & pose);
    void AddTruePose(boost::shared_ptr<const sfl::Frame> pose);
    
    const std::string & GetName() const;
    const sfl::Frame & GetTruePose() const;
    const Object & GetBody() const;
    const trajectory_t & GetTrueTrajectory() const;
    size_t GetIdentifier() const;
    const World & GetWorld() const;
    boost::shared_ptr<const Lidar> GetLidar(int channel) const;
    boost::shared_ptr<const Sharp> GetSharp(int channel) const;
    boost::shared_ptr<HAL> GetHAL();
    boost::shared_ptr<RobotDescriptor> GetDescriptor();
    
    void AddLine(const sfl::Line & line);
    void AddDrawing(boost::shared_ptr<Drawing> drawing);
    void AddDrawing(Drawing * drawing);
    void AddCamera(boost::shared_ptr<Camera> camera);
    void AddCamera(Camera * camera);
    
    boost::shared_ptr<Lidar>
    DefineLidar(const sfl::Frame & mount, size_t nscans, double rhomax,
		double phi0, double phirange, int hal_channel);

    boost::shared_ptr<Lidar>
    DefineLidar(boost::shared_ptr<sfl::Scanner> scanner);

    boost::shared_ptr<Sharp>
    DefineSharp(const sfl::Frame & mount, double rmax, int channel);
    
    boost::shared_ptr<DiffDrive>
    DefineDiffDrive(double wheelbase, double wheelradius);
    
    boost::shared_ptr<HoloDrive>
    DefineHoloDrive(double axislength);

    boost::shared_ptr<BicycleDrive>
    DefineBicycleDrive(double wheelbase, double wheelradius, double axlewidth);
    
  private:    
    static size_t next_identifier;
    const size_t m_identifier;
    const bool m_enable_trajectory;
    const World & m_world;
    boost::shared_ptr<HAL> m_hal;
    boost::shared_ptr<RobotDescriptor> m_descriptor;
    std::vector<boost::shared_ptr<Drawing> > m_drawing;
    std::vector<boost::shared_ptr<Camera> > m_camera;
    std::vector<boost::shared_ptr<Sensor> > m_sensor;
    boost::shared_ptr<Drive> m_drive;
    std::map<int, boost::shared_ptr<Lidar> > m_lidar;
    std::map<int, boost::shared_ptr<Sharp> > m_sharp;
    boost::shared_ptr<Object> m_body;
    trajectory_t m_true_trajectory;
    boost::shared_ptr<sfl::Frame> m_true_pose;
  };
  
}

#endif // NPM_ROBOTSERVER_HPP
