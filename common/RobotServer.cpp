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
#include "RobotDescriptor.hpp"
#include "HAL.hpp"
#include "Object.hpp"
#include "RobotDrawing.hpp"
#include "TrajectoryDrawing.hpp"
#include "RobotZoomCamera.hpp"
#include "Sharp.hpp"
#include "Lidar.hpp"
#include "DiffDrive.hpp"
#include "HoloDrive.hpp"
#include "BicycleDrive.hpp"
#include "DiffDriveDrawing.hpp"
#include "HoloDriveDrawing.hpp"
#include "BicycleDriveDrawing.hpp"
#include <sfl/util/Frame.hpp>
#include <sfl/util/Pthread.hpp>
#include <sfl/api/Scanner.hpp>
#include <iostream>
#include <sstream>
#include <cmath>


using namespace sfl;
using namespace boost;
using namespace std;


namespace npm {
  
  
  size_t RobotServer::next_identifier(0);
  
  
  RobotServer::
  RobotServer(shared_ptr<RobotDescriptor> descriptor, const World & world,
	      bool enable_trajectory)
    : m_identifier(next_identifier++),
      m_enable_trajectory(enable_trajectory),
      m_world(world),
      m_hal(new npm::HAL(this)),
      m_descriptor(descriptor),
      m_body(new Object(descriptor->name)),
      m_true_pose(new Frame())
  {
    AddDrawing(new RobotDrawing(this));
    AddDrawing(new TrajectoryDrawing(this));
    double zoom(2);
    if(descriptor->GetOption("camera_zoom") != ""){
      istringstream is(descriptor->GetOption("camera_zoom"));
      if( ! (is >> zoom)){
	cerr << "WARNING: cannot read camera_zoom from \""
	     << descriptor->GetOption("camera_zoom") << "\"\n";
	zoom = 2;
      }
    }
    AddCamera(new RobotZoomCamera(this, zoom));
  }
  
  
  shared_ptr<Lidar> RobotServer::
  DefineLidar(const Frame & mount, size_t nscans, double rhomax,
	      double phi0, double phirange, int hal_channel)
  {
    ostringstream os;
    os << m_descriptor->name << "-lidar-" << hal_channel;
    shared_ptr<Mutex> mutex(Mutex::Create(os.str()));
    if( ! mutex){
      cerr << "sfl::Mutex::Create() failed in RobotServer::DefineLidar()\n";
      exit(EXIT_FAILURE);
    }
    shared_ptr<Scanner>
      scanner(new Scanner(GetHAL(), hal_channel, mount, nscans,
			  rhomax, phi0, phirange, mutex));
    return DefineLidar(scanner);
  }
  
  
  shared_ptr<Lidar> RobotServer::
  DefineLidar(shared_ptr<Scanner> scanner)
  {
    if(m_lidar.end() != m_lidar.find(scanner->hal_channel)){
      cerr << "hal_channel " << scanner->hal_channel
	   << " already taken in RobotServer::DefineLidar()\n";
      exit(EXIT_FAILURE);
    }
    shared_ptr<Lidar>
      lidar(new Lidar(this, GetHAL(), scanner->hal_channel, *scanner->mount,
		      scanner->nscans, scanner->rhomax, scanner->phi0,
		      scanner->phirange, scanner));
    m_lidar.insert(make_pair(scanner->hal_channel, lidar));
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
    AddDrawing(new DiffDriveDrawing(m_descriptor->name + "_drive", dd));
    return dd;
  }
  
  
  shared_ptr<HoloDrive> RobotServer::
  DefineHoloDrive(double axislength)
  {
    if(m_drive)
      return shared_ptr<HoloDrive>();
    shared_ptr<HoloDrive> hd(new HoloDrive(GetHAL(), axislength));
    m_drive = hd;
    AddDrawing(new HoloDriveDrawing(m_descriptor->name + "_drive", hd));
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
    AddDrawing(new BicycleDriveDrawing(m_descriptor->name + "_drive", hd));
    return hd;
  }
  
  
  void RobotServer::
  AddLine(const Line & line)
  {
    m_body->AddLine(line);
  }
  
  
  void RobotServer::
  AddDrawing(shared_ptr<Drawing> drawing)
  {
    m_drawing.push_back(drawing);
  }
  
  
  void RobotServer::
  AddDrawing(Drawing * drawing)
  {
    m_drawing.push_back(shared_ptr<Drawing>(drawing));
  }
  
  
  void RobotServer::
  AddCamera(shared_ptr<Camera> camera)
  {
    m_camera.push_back(camera);
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
  
  
  shared_ptr<RobotDescriptor> RobotServer::
  GetDescriptor()
  {
    return m_descriptor;
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
      m_body->UpdateSensor(sensor);
  }
  
  
  void RobotServer::
  SimulateAction(double timestep)
  {
    m_hal->UpdateSpeeds();
    if(m_drive)
      AddTruePose(m_drive->NextPose( * m_true_pose, timestep));
  }
  
  
  void RobotServer::
  InitializeTruePose(const Frame & pose)
  {
    m_true_trajectory.clear();
    AddTruePose(shared_ptr<Frame>(new Frame(pose)));
  }
  
  
  void RobotServer::
  AddTruePose(shared_ptr<const Frame> pose)
  {
    m_true_pose->Set( * pose);
    if(m_enable_trajectory)
      m_true_trajectory.push_back(pose);
    m_body->TransformTo( * m_true_pose);
  }
  
  
  const string & RobotServer::
  GetName() const
  {
    return m_descriptor->name;
  }
  
  
  const Frame & RobotServer::
  GetTruePose() const
  {
    return  * m_true_pose;
  }
  
  
  const Object & RobotServer::
  GetBody() const
  {
    return * m_body;
  }
  
  
  const RobotServer::trajectory_t & RobotServer::
  GetTrueTrajectory() const
  {
    return m_true_trajectory;
  }
  
  
  size_t RobotServer::
  GetIdentifier() const
  {
    return m_identifier;
  }
  
  
  const World & RobotServer::
  GetWorld() const
  {
    return m_world;
  }
  
  
  shared_ptr<const Lidar> RobotServer::
  GetLidar(int channel) const
  {
    map<int, shared_ptr<Lidar> >::const_iterator il(m_lidar.find(channel));
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
  
}
