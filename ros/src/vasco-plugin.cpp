/* 
 * Copyright (C) 2014 Roland Philippsen. All rights resevred.
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

#include <npm/Plugin.hpp>
#include <npm/Factory.hpp>
#include <npm/ext/Zombie.hpp>
#include <npm/RobotServer.hpp>
#include <npm/Lidar.hpp>

#include "ros/ros.h"
#include "sfl2/Scan.h"

using namespace std;


class VascoRobot
  : public npm::LidarZombie
{
public:
  VascoRobot (string const & name)
    : LidarZombie (name),
      scan_topic_ ("scan"),
      server_ (0)
  {
    reflectParameter ("scan_topic", &scan_topic_);
  }
  
  
  virtual bool Initialize (npm::RobotServer &server)
  {
    if ( ! LidarZombie::Initialize (server)) {
      return false;
    }
    ros::NodeHandle node;
    scan_pub_ = node.advertise <sfl2::Scan> (scan_topic_, 1);
    server_ = &server;
    return true;
  }
  
  
  virtual bool PrepareAction (double timestep)
  {
    if ( ! LidarZombie::PrepareAction (timestep)) {
      return false;
    }
    
    //// in case we want to receive messages as well...
    //
    // if ( ! ros::ok()) {
    //   return false;
    // }
    
    sfl::Frame const & pose (server_->GetPose());
    
    sfl2::Scan msg;
    msg.px = pose.X();
    msg.py = pose.Y();
    msg.pth = pose.Theta();
    msg.mx = m_lidar->mount->X();
    msg.my = m_lidar->mount->Y();
    msg.mth = m_lidar->mount->Theta();
    msg.nscans = m_lidar->nscans;
    msg.rhomax = m_lidar->rhomax;
    msg.phi0 = m_lidar->phi0;
    msg.phirange = m_lidar->phirange;
    for (size_t ii (0); ii < m_lidar->nscans; ++ii) {
      msg.rho.push_back (m_lidar->GetNoisyRho (ii));
    }
    scan_pub_.publish (msg);
    
    ros::spinOnce();
    
    return true;
  }
  
  
  std::string scan_topic_;
  ros::Publisher scan_pub_;
  npm::RobotServer * server_;
};


int npm_plugin_init ()
{
  if ( ! ros::isInitialized()) {
    int argc = 0;
    char** argv = NULL;
    ros::init(argc, argv, "sfl2",
	      ros::init_options::NoSigintHandler |
	      ros::init_options::AnonymousName);
  }
  
  npm::Factory::Instance().declare<VascoRobot>("VascoRobot");
  
  return 0;
}
