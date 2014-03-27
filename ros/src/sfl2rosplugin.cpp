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
#include <sfl/api/Scanner.hpp>
#include <iostream>

#include "ros/ros.h"
#include "sfl2ros/Scan.h"


class ROSZombie
  : public npm::LidarZombie
{
public:
  explicit ROSZombie (std::string const & name)
    : npm::LidarZombie (name)
  {
    ros::NodeHandle node;
    scan_pub_ = node.advertise<sfl2ros::Scan> (name + "_scan", 1);
  }
  
  virtual bool PrepareAction(double timestep)
  {
    if ( ! npm::LidarZombie::PrepareAction (timestep)) {
      return false;
    }
    publish ();
    return true;
  }
  
  void publish ()
  {
    sfl2ros::Scan msg;
    for (size_t ii (0); ii < m_scanner->nscans; ++ii) {
      double rho;
      m_scanner->Rho (ii, rho);
      msg.rho.push_back (rho);
    }
    scan_pub_.publish (msg);
  }
  
private:
  ros::Publisher scan_pub_;
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
  
  npm::Factory::Instance().declare<ROSZombie>("ROSZombie");
  
  return 0;
}
