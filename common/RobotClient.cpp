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


#include "RobotClient.hpp"
#include "RobotServer.hpp"
#include <sfl/util/Frame.hpp>


using namespace sfl;
using namespace boost;


namespace npm {

  
  RobotClient::
  RobotClient(boost::shared_ptr<RobotDescriptor> descriptor,
	      const World & world, bool enable_trajectory)
    : m_server(new RobotServer(descriptor, world, enable_trajectory))
  {
  }
  
  
  RobotClient::
  ~RobotClient()
  {
  }


  void RobotClient::
  GetPose(double & x, double & y, double & theta)
  {
    const Frame pose(m_server->GetTruePose());
    x = pose.X();
    y = pose.Y();
    theta = pose.Theta();
  }
  
  
  shared_ptr<const Goal> RobotClient::
  GetGoal()
  {
    return shared_ptr<const Goal>();
  }


  shared_ptr<HAL> RobotClient::
  GetHAL()
  {
    return m_server->GetHAL();
  }
  
  
  shared_ptr<RobotDescriptor> RobotClient::
  GetDescriptor()
  {
    return m_server->GetDescriptor();
  }
  
  
  void RobotClient::
  AddLine(const Line & line)
  {
    m_server->AddLine(line);
  }
  
  
  void RobotClient::
  AddDrawing(shared_ptr<Drawing> drawing)
  {
    m_server->AddDrawing(drawing);
  }
  
  
  void RobotClient::
  AddDrawing(Drawing * drawing)
  {
    m_server->AddDrawing(drawing);
  }
  
  
  void RobotClient::
  AddCamera(shared_ptr<Camera> camera)
  {
    m_server->AddCamera(camera);
  }
  
  
  void RobotClient::
  AddCamera(Camera * camera)
  {
    m_server->AddCamera(camera);
  }
  
  
  shared_ptr<Lidar> RobotClient::
  DefineLidar(const Frame & mount, size_t nscans, double rhomax,
	      double phi0, double phirange, int hal_channel)
  {
    return m_server->DefineLidar(mount, nscans, rhomax, phi0,
				 phirange, hal_channel);
  }
  
  
  shared_ptr<Lidar> RobotClient::
  DefineLidar(shared_ptr<Scanner> scanner)
  {
    return m_server->DefineLidar(scanner);
  }
  
  
  shared_ptr<Sharp> RobotClient::
  DefineSharp(const Frame & mount, double rmax, int channel)
  {
    return m_server->DefineSharp(mount, rmax, channel);
  }
  
  
  shared_ptr<DiffDrive> RobotClient::
  DefineDiffDrive(double wheelbase, double wheelradius)
  {
    return m_server->DefineDiffDrive(wheelbase, wheelradius);
  }
  
  
  shared_ptr<HoloDrive> RobotClient::
  DefineHoloDrive(double axislength)
  {
    return m_server->DefineHoloDrive(axislength);
  }


  shared_ptr<BicycleDrive> RobotClient::
  DefineBicycleDrive(double wheelbase, double wheelradius)
  {
    return m_server->DefineBicycleDrive(wheelbase, wheelradius);
  }
  
}
