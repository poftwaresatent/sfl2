/* 
 * Copyright (C) 2004
 * Swiss Federal Institute of Technology, Lausanne. All rights reserved.
 * 
 * Author: Roland Philippsen <roland dot philippsen at gmx dot net>
 *         Autonomous Systems Lab <http://asl.epfl.ch/>
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


#include "RobotServer.hpp"
#include "RobotClient.hpp"
#include "HAL.hpp"
#include "Object.hpp"
#include "gfx/RobotDrawing.hpp"
#include "gfx/TrajectoryDrawing.hpp"
#include "gfx/RobotZoomCamera.hpp"
#include "Sharp.hpp"
#include "Lidar.hpp"
#include "DiffDrive.hpp"
#include "HoloDrive.hpp"
#include "BicycleDrive.hpp"
#include "gfx/DiffDriveDrawing.hpp"
#include "gfx/HoloDriveDrawing.hpp"
#include "gfx/BicycleDriveDrawing.hpp"
#include "pdebug.hpp"
#include "NoiseModel.hpp"
#include "World.hpp"
#include "CheatSheet.hpp"
#include <sfl/util/strutil.hpp>
#include <sfl/util/Frame.hpp>
#include <sfl/api/Scanner.hpp>
#include <sfl/api/Pose.hpp>
#include <iostream>
#include <sstream>
#include <cmath>

// rfct
#include <sys/time.h>


using namespace sfl;
using namespace boost;
using namespace std;


namespace {
  
  class rfct
    : public sfl::LocalizationInterface
  {
  public:
    explicit rfct(npm::RobotServer const * server): m_server(server) {}
    
    virtual void GetPose (sfl::Pose & pose) {
      struct timeval tv;
      gettimeofday(&tv, 0);
      sfl::timespec_t ts;
      TIMEVAL_TO_TIMESPEC(&tv, &ts);
      
      pose.Set(m_server->GetTruePose().X(),
	       m_server->GetTruePose().Y(),
	       m_server->GetTruePose().Theta(),
	       ts,
	       1.0, 1.0, 1.0,
	       0.0, 0.0, 0.0);
    }
    
    npm::RobotServer const * m_server;
  };
  
}

namespace npm {
  
  
  RobotServer::
  RobotServer(RobotClient *client,
	      const World & world)
    : m_client(client),
      m_world(world),
      m_hal(new HAL(this)),
      m_true_body(new Object(client->name, "true_body")),
      m_true_pose(new Frame())
  {
    if(client->m_noisy_odometry){
      m_noisy_pose.reset(new Frame());
      m_noisy_body.reset(new Object(client->name, "noisy_body"));
      m_noisy_trajectory.reset(new trajectory_t());
      m_odometry_noise_model.reset(new NoiseModel(client->m_odometry_noise_min_factor,
						  client->m_odometry_noise_max_factor,
						  client->m_odometry_noise_min_offset,
						  client->m_odometry_noise_max_offset));
    }
    
    if(client->m_noisy_scanners){
      m_scanner_noise_model.reset(new NoiseModel(client->m_scanner_noise_min_factor,
						 client->m_scanner_noise_max_factor,
						 client->m_scanner_noise_min_offset,
						 client->m_scanner_noise_max_offset));
    }
    
    AddDrawing(new RobotDrawing(this, client->GetColor()));
    AddDrawing(new TrajectoryDrawing(this));
    AddCamera(new RobotZoomCamera(this, client->m_camera_zoom));
  }
  
  
  shared_ptr<Lidar> RobotServer::
  DefineLidar(const sfl::Frame & mount, std::string const & name,
	      size_t nscans, double rhomax, double phi0, double phirange)
  {
    if (m_lidar.end() != m_lidar.find(name)) {
      cerr << "lidar name " << name
	   << " already taken in RobotServer::DefineLidar()\n";
      exit(EXIT_FAILURE);
    }
    shared_ptr<Lidar>
      lidar(new Lidar(this, name, mount, nscans, rhomax, phi0, phirange,
		      m_scanner_noise_model));
    m_lidar.insert(make_pair(name, lidar));
    m_sensor.push_back(lidar);
    return lidar;
  }
  
  
  shared_ptr<Sharp> RobotServer::
  DefineSharp(const Frame & mount, double rmax, int channel)
  {
    shared_ptr<Sharp> sharp(new Sharp(this, mount, rmax, channel));
    m_sharp.insert(make_pair(channel, sharp));
    m_sensor.push_back(sharp);
    return sharp;
  }
  
  
  shared_ptr<DiffDrive> RobotServer::
  DefineDiffDrive(double wheelbase, double wheelradius)
  {
    if(m_drive)
      return shared_ptr<DiffDrive>();
    shared_ptr<DiffDrive> dd(new DiffDrive(GetHAL(), wheelbase, wheelradius));
    m_drive = dd;
    AddDrawing(new DiffDriveDrawing(m_client->name + "_drive", dd));
    return dd;
  }
  
  
  shared_ptr<HoloDrive> RobotServer::
  DefineHoloDrive(double axislength)
  {
    if(m_drive)
      return shared_ptr<HoloDrive>();
    shared_ptr<HoloDrive> hd(new HoloDrive(GetHAL(), axislength));
    m_drive = hd;
    AddDrawing(new HoloDriveDrawing(m_client->name + "_drive", hd));
    return hd;
  }

  shared_ptr<BicycleDrive> RobotServer::
  DefineBicycleDrive(double wheelbase, double wheelradius, double axlewidth)
  {
    if(m_drive)
      return shared_ptr<BicycleDrive>();
    shared_ptr<BicycleDrive>
      hd(new BicycleDrive(GetHAL(), wheelbase, wheelradius, axlewidth));
    m_drive = hd;
    AddDrawing(new BicycleDriveDrawing(m_client->name + "_drive", hd));
    return hd;
  }
  
  
  void RobotServer::
  AddLine(double x0, double y0, double x1, double y1)
  {
    AddLine(Line(x0, y0, x1, y1));
  }
  
  
  void RobotServer::
  AddLine(const Line & line)
  {
    m_true_body->AddLine(line);
    if(m_noisy_body)
      m_noisy_body->AddLine(line);      
  }
  
  
  void RobotServer::
  AddDrawing(Drawing * drawing)
  {
    m_drawing.push_back(shared_ptr<Drawing>(drawing));
  }
  
  
  void RobotServer::
  AddCamera(Camera * camera)
  {
    m_camera.push_back(shared_ptr<Camera>(camera));
  }
  
  
  shared_ptr<npm::HAL> RobotServer::
  GetHAL()
  {
    return m_hal;
  }
  
  
  shared_ptr<sfl::LocalizationInterface> RobotServer::
  CreateFakeLocalization() const
  {
    return shared_ptr<rfct>(new rfct(this));
  }
  
  
  void RobotServer::
  UpdateAllSensors()
  {
    for(size_t ii(0); ii < m_sensor.size(); ++ii)
      m_sensor[ii]->Update();
  }
  
  
  void RobotServer::
  UpdateSensor(Sensor & sensor) const
  {
    if(sensor.owner != this)
      m_true_body->UpdateSensor(sensor);
  }
  
  
  void RobotServer::
  SimulateAction(double timestep)
  {
    m_hal->UpdateSpeeds();
    if( ! m_drive)
      return;
    AddTruePose(m_drive->NextPose( * m_true_pose, timestep));
    if( ! m_noisy_pose)
      return;
    m_hal->EnableOdometryNoise(m_odometry_noise_model.get());
    AddNoisyPose(m_drive->NextPose( * m_noisy_pose, timestep));
    m_hal->DisableOdometryNoise();
  }
  
  
  void RobotServer::
  InitializePose(const Frame & pose)
  {
    m_true_trajectory.clear();
    AddTruePose(shared_ptr<Frame>(new Frame(pose)));
    if(m_noisy_trajectory)
      m_noisy_trajectory->clear();
    AddNoisyPose(shared_ptr<Frame>(new Frame(pose)));
  }
  
  
  void RobotServer::
  AddPose(shared_ptr<const Frame> pose)
  {
    if(m_noisy_pose)
      AddNoisyPose(pose);
    else
      AddTruePose(pose);
  }
  
  
  void RobotServer::
  AddTruePose(shared_ptr<const Frame> pose)
  {
    m_true_pose->Set( * pose);
    if(m_client->m_enable_trajectory)
      m_true_trajectory.push_back(pose);
    m_true_body->TransformTo( * m_true_pose);
  }
  
  
  void RobotServer::
  AddNoisyPose(shared_ptr<const Frame> pose)
  {
    if( ! m_noisy_pose)
      return;
    m_noisy_pose->Set( * pose);
    if(m_client->m_enable_trajectory && m_noisy_trajectory)
      m_noisy_trajectory->push_back(pose);
    if(m_noisy_body)
      m_noisy_body->TransformTo( * m_noisy_pose);
  }
  
  
  const string & RobotServer::
  GetName() const
  {
    return m_client->name;
  }
  
  
  const Frame & RobotServer::
  GetTruePose() const
  {
    return  * m_true_pose;
  }
  
  
  const Frame * RobotServer::
  GetNoisyPose() const
  {
    return m_noisy_pose.get();
  }
  
  
  const Object & RobotServer::
  GetBody() const
  {
    return * m_true_body;
  }
  
  
  const Object * RobotServer::
  GetNoisyBody() const
  {
    return m_noisy_body.get();
  }
  
  
  const RobotServer::trajectory_t & RobotServer::
  GetTrueTrajectory() const
  {
    return m_true_trajectory;
  }
  
  
  const RobotServer::trajectory_t * RobotServer::
  GetNoisyTrajectory() const
  {
    return m_noisy_trajectory.get();
  }
  
  
  const World & RobotServer::
  GetWorld() const
  {
    return m_world;
  }
  
  
  shared_ptr<const Lidar> RobotServer::
  GetLidar(std::string const & name) const
  {
    map<string, shared_ptr<Lidar> >::const_iterator il(m_lidar.find(name));
    if(m_lidar.end() == il)
      return shared_ptr<const Lidar>();
    return il->second;
  }
  
  
  shared_ptr<const Sharp> RobotServer::
  GetSharp(int channel) const
  {
    map<int, shared_ptr<Sharp> >::const_iterator is(m_sharp.find(channel));
    if(m_sharp.end() == is)
      return shared_ptr<const Sharp>();
    return is->second;
  }
  
  
  void RobotServer::
  AddKeyListener(boost::shared_ptr<KeyListener> listener) const
  {
    m_world.AddKeyListener(listener);
  }
  
  
  boost::shared_ptr<npm::CheatSheet> RobotServer::
  CreateCheatSheet() const
  {
    boost::shared_ptr<npm::CheatSheet> cs(new CheatSheet(&m_world, this));
    return cs;
  }
  
}
