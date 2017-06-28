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


#include <memory>
#include <vector>
#include <map>


namespace sfl {
  class Line;
  class Frame;
  class LocalizationInterface;
}


namespace npm {

  class RobotClient;  
  class Sensor;
  class Object;
  class World;
  class Lidar;
  class Sharp;
  class Drawing;
  class Camera;
  class Drive;
  class DiffDrive;
  class HoloDrive;
  class BicycleDrive;
  class NoiseModel;
  class KeyListener;
  class CheatSheet;
  
  
  class RobotServer
  {
  public:
    typedef std::vector<std::shared_ptr<const sfl::Frame> > trajectory_t;
    
    RobotServer(RobotClient *client,
		World const &world);
    
    void UpdateAllSensors();
    void UpdateSensor(Sensor & sensor) const;
    void SimulateAction(double timestep);
    void InitializePose(const sfl::Frame & pose);
    void AddPose(std::shared_ptr<const sfl::Frame> pose);
    
  public:
    const std::string & GetName() const;
    const sfl::Frame & GetPose() const;
    const Object & GetBody() const;
    const trajectory_t & GetTrajectory() const;
    const World & GetWorld() const;
    std::shared_ptr<const Lidar> GetLidar(std::string const & name) const;
    std::shared_ptr<const Sharp> GetSharp(int channel) const;
    std::shared_ptr<sfl::LocalizationInterface> CreateFakeLocalization() const;
    
    void AddLine(double x0, double y0, double x1, double y1);
    void AddLine(const sfl::Line & line);

  private:
    void AddDrawing(Drawing * drawing);
    void AddCamera(Camera * camera);
    
  public:
    
    std::shared_ptr<Lidar>
    DefineLidar(const sfl::Frame & mount, std::string const & name,
		size_t nscans, double rhomax, double phi0, double phirange);
    
    std::shared_ptr<Sharp>
    DefineSharp(const sfl::Frame & mount, double rmax, int channel);
    
    std::shared_ptr<DiffDrive>
    DefineDiffDrive(double wheelbase, double wheelradius);
    
    std::shared_ptr<HoloDrive>
    DefineHoloDrive(double axislength);

    std::shared_ptr<BicycleDrive>
    DefineBicycleDrive(double wheelbase, double wheelradius, double axlewidth);
    
    void AddKeyListener(std::shared_ptr<KeyListener> listener) const;
    std::shared_ptr<npm::CheatSheet> CreateCheatSheet() const;
    
    RobotClient const * GetClient() const { return m_client; }
    
  private:
    friend class Simulator;
    
    RobotClient *m_client;
    World const &m_world;
    std::vector<std::shared_ptr<Drawing> > m_drawing;
    std::vector<std::shared_ptr<Camera> > m_camera;
    std::vector<std::shared_ptr<Sensor> > m_sensor;
    std::shared_ptr<Drive> m_drive;
    std::map<std::string, std::shared_ptr<Lidar> > m_lidar;
    std::map<int, std::shared_ptr<Sharp> > m_sharp;
    std::shared_ptr<Object> m_body;
    trajectory_t m_trajectory;
    std::shared_ptr<sfl::Frame> m_pose;
    std::shared_ptr<NoiseModel> m_scanner_noise_model;
  };
  
}

#endif // NPM_ROBOTSERVER_HPP
