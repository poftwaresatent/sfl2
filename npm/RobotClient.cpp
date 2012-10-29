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
#include <sfl/util/Line.hpp>
#include <sfl/util/Polygon.hpp>


using namespace sfl;
using namespace boost;


namespace npm {

  
  RobotClient::
  RobotClient(std::string const &name)
    : fpplib::Configurable(name),
      m_enable_trajectory(true),
      m_noisy_odometry(false),
      m_odometry_noise_min_factor(0.95), // factors <0 are ignored
      m_odometry_noise_max_factor(1.05),
      m_odometry_noise_min_offset(1), // if min>max then offsets are ignored
      m_odometry_noise_max_offset(-1),
      m_noisy_scanners(false),
      m_scanner_noise_min_factor(-1), // factors <0 are ignored
      m_scanner_noise_max_factor(-1),
      m_scanner_noise_min_offset(-0.1), // if min>max then offsets are ignored
      m_scanner_noise_max_offset( 0.1),
      m_camera_zoom(2)

  {
    reflectParameter("enable_trajectory", &m_enable_trajectory);
    reflectParameter("noisy_odometry", &m_noisy_odometry);
    reflectParameter("odometry_noise_min_factor", &m_odometry_noise_min_factor);
    reflectParameter("odometry_noise_max_factor", &m_odometry_noise_max_factor);
    reflectParameter("odometry_noise_min_offset", &m_odometry_noise_min_offset);
    reflectParameter("odometry_noise_max_offset", &m_odometry_noise_max_offset);
    reflectParameter("noisy_scanners", &m_noisy_scanners);
    reflectParameter("scanner_noise_min_factor", &m_scanner_noise_min_factor);
    reflectParameter("scanner_noise_max_factor", &m_scanner_noise_max_factor);
    reflectParameter("scanner_noise_min_offset", &m_scanner_noise_min_offset);
    reflectParameter("scanner_noise_max_offset", &m_scanner_noise_max_offset);
    reflectParameter("camera_zoom", &m_camera_zoom);
  }
  
  
  bool RobotClient::
  Initialize(RobotServer &server)
  {
    m_hal = server.GetHAL();
    return true;
  }

  
  
  shared_ptr<const Goal> RobotClient::
  GetGoal()
  {
    return shared_ptr<const Goal>();
  }


  // void RobotClient::
  // AddLine(double x0, double y0, double x1, double y1)
  // {
  //   m_server->AddLine(Line(x0, y0, x1, y1));
  // }
  
  
  // void RobotClient::
  // AddLine(const Line & line)
  // {
  //   m_server->AddLine(line);
  // }
  
  
  // void RobotClient::
  // AddPolygon(const Polygon & polygon)
  // {
  //   size_t const npoints(polygon.GetNPoints());
  //   if (npoints < 2)
  //     return;
  //   Point const * pt0(polygon.GetPoint(npoints - 1));
  //   Point const * pt1(polygon.GetPoint(0));
  //   m_server->AddLine(Line(pt0->X(), pt0->Y(), pt1->X(), pt1->Y()));
  //   for (size_t ii(1); ii < npoints; ++ii) {
  //     pt0 = pt1;
  //     pt1 = polygon.GetPoint(ii);
  //     m_server->AddLine(Line(pt0->X(), pt0->Y(), pt1->X(), pt1->Y()));
  //   }
  // }
  
  
  // void RobotClient::
  // AddDrawing(shared_ptr<Drawing> drawing)
  // {
  //   m_server->AddDrawing(drawing);
  // }
  
  
  // void RobotClient::
  // AddDrawing(Drawing * drawing)
  // {
  //   m_server->AddDrawing(drawing);
  // }
  
  
  // void RobotClient::
  // AddCamera(shared_ptr<Camera> camera)
  // {
  //   m_server->AddCamera(camera);
  // }
  
  
  // void RobotClient::
  // AddCamera(Camera * camera)
  // {
  //   m_server->AddCamera(camera);
  // }
  
  
  // shared_ptr<Lidar> RobotClient::
  // DefineLidar(const Frame & mount, size_t nscans, double rhomax,
  // 	      double phi0, double phirange, int hal_channel)
  // {
  //   return m_server->DefineLidar(mount, nscans, rhomax, phi0,
  // 				 phirange, hal_channel);
  // }
  
  
  // shared_ptr<Lidar> RobotClient::
  // DefineLidar(shared_ptr<Scanner> scanner)
  // {
  //   return m_server->DefineLidar(scanner);
  // }
  
  
  // shared_ptr<Sharp> RobotClient::
  // DefineSharp(const Frame & mount, double rmax, int channel)
  // {
  //   return m_server->DefineSharp(mount, rmax, channel);
  // }
  
  
  // shared_ptr<DiffDrive> RobotClient::
  // DefineDiffDrive(double wheelbase, double wheelradius)
  // {
  //   return m_server->DefineDiffDrive(wheelbase, wheelradius);
  // }
  
  
  // shared_ptr<HoloDrive> RobotClient::
  // DefineHoloDrive(double axislength)
  // {
  //   return m_server->DefineHoloDrive(axislength);
  // }


  // shared_ptr<BicycleDrive> RobotClient::
  // DefineBicycleDrive(double wheelbase, double wheelradius, double axlebase)
  // {
  //   return m_server->DefineBicycleDrive(wheelbase, wheelradius, axlebase);
  // }
  
}
