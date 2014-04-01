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
#include <npm/RobotClient.hpp>
#include <npm/RobotServer.hpp>
#include <npm/HAL.hpp>
#include <sfl/util/numeric.hpp>
#include <iostream>
#include <cmath>

#include "ros/ros.h"
#include "cargo_ants_msgs/VehicleState.h"
#include "cargo_ants_msgs/Trajectory.h"


using namespace cargo_ants_msgs;


class CargoANTsMockup
  : public npm::RobotClient
{
public:
  
  explicit CargoANTsMockup (std::string const & name)
    : npm::RobotClient (name),
      width_ (2.0),
      length_ (4.0),
      align_distance_ (1.5),
      align_heading_ (0.3),
      vrot_ (45 * M_PI / 180),
      vtrans_ (2.0),
      trajectory_topic_ ("trajectory"),
      vehicle_state_topic_ ("vehicle_state"),
      msg_queue_size_ (10),
      server_ (0)
  {
    reflectParameter ("width", &width_);
    reflectParameter ("length", &length_);
    reflectParameter ("align_distance", &align_distance_);
    reflectParameter ("align_heading", &align_heading_);
    reflectParameter ("vrot", &vrot_);
    reflectParameter ("vtrans", &vtrans_);
    reflectParameter ("trajectory_topic", &trajectory_topic_);
    reflectParameter ("vehicle_state_topic", &vehicle_state_topic_);
    reflectParameter ("msg_queue_size", &msg_queue_size_);
  }
  
  
  virtual bool Initialize (npm::RobotServer & server)
  {
    if ( ! npm::RobotClient::Initialize(server)) {
      return false;
    }
    
    server_ = &server;		// to get at simulator-internal data
    drive_ = server.DefineHoloDrive(0.8 * width_);
    
    server.AddLine (-length_/2, -width_/2, -length_/2,  width_/2);
    server.AddLine (-length_/2,  width_/2,  length_/2,  width_/2);
    server.AddLine ( length_/2,  width_/2,  length_/2, -width_/2);
    server.AddLine ( length_/2, -width_/2, -length_/2, -width_/2);
    
    ros::NodeHandle node;
    trajectory_sub_ = node.subscribe (trajectory_topic_, msg_queue_size_,
				      &CargoANTsMockup::trajectoryCB, this);
    vehicle_state_pub_ = node.advertise <VehicleState> (vehicle_state_topic_, 1);
    
    return true;
  }
  
  
  virtual bool PrepareAction (double timestep)
  {
    // I wonder how many times we should spin when we will be
    // subscribed to many topics and possibly running much slower than
    // realtime... for example when the simulation is paused.  Maybe
    // we'll need to spawn a separate thread, but it would be nice to
    // avoid that.
    
    if ( ! ros::ok()) {
      return false;
    }
    ros::spinOnce();
    
    if ( ! trajectory_.empty()) {
      if ( ! move (trajectory_.back(), timestep)) {
	return false;
      }
    }
    
    publish ();
    
    return true;
  }
  
  
  virtual void InitPose (sfl::Pose const & pose)
  {
  }
  
  
  virtual void SetPose (sfl::Pose const & pose)
  {
  }
  
  virtual bool GetPose (sfl::Pose & pose)
  {
    return false;
  }
  
  
  virtual void SetGoal (double timestep, const sfl::Goal & goal)
  {
  }
  
  
  virtual bool GetGoal (sfl::Goal & goal)
  {
    if (trajectory_.empty()) {
      return false;
    }
    TrajectoryPoint const & tp (trajectory_.back());
    goal.Set (tp.x, tp.y, tp.th, 0.1, 0.1);
    return true;
  }
  
  
  virtual bool GoalReached ()
  {
    return false;
  }
  
  
  void trajectoryCB (Trajectory::ConstPtr const & msg)
  {
    trajectory_ = msg->points;
  }
  
  
  bool move (TrajectoryPoint const & target, double timestep)
  {
    sfl::Frame target_pose (target.x, target.y, target.th);
    sfl::Frame const & pose (server_->GetTruePose());
    pose.From (target_pose);
    double const dhead (atan2 (target_pose.Y(), target_pose.X()));
    double const dist (sqrt (pow (target_pose.X(), 2) + pow (target_pose.Y(), 2)));
    
    double qd[3];
    size_t len (3);
    if (dist > align_distance_) {
      if (fabs (dhead) > align_heading_) {
	qd[0] = 0.0;
	qd[1] = 0.0;
	qd[2] = sfl::boundval (-vrot_, dhead / timestep, vrot_);
	
	std::cerr << name << target_pose << "  " << dhead << " --> align " << qd[2] << "\n";
      }
      else {
	qd[0] = sfl::boundval (-vtrans_, dist  / timestep, vtrans_);
	qd[1] = 0.0;
	qd[2] = sfl::boundval (-vrot_,   dhead / timestep, vrot_);
	
	std::cerr << name << target_pose << "  " << dhead << " --> advance " << qd[0] << "  " << qd[1] << "  " << qd[2] << "\n";
      }
    }
    else {
      qd[0] = sfl::boundval (-vtrans_, target_pose.X()     / timestep, vtrans_);
      qd[1] = sfl::boundval (-vtrans_, target_pose.Y()     / timestep, vtrans_);
      qd[2] = sfl::boundval (-vrot_,   target_pose.Theta() / timestep, vrot_);
      
      std::cerr << name << target_pose << "  " << dhead << " --> adjust " << qd[0] << "  " << qd[1] << "  " << qd[2] << "\n";
    }
    
    return (0 == m_hal->speed_set (qd, &len)) && (3 == len);
  }
  
  
  void publish ()
  {
    VehicleState vehicle_state;
    sfl::Frame const & pose (server_->GetTruePose());
    vehicle_state.location.x = pose.X();
    vehicle_state.location.y = pose.Y();
    vehicle_state.location.z = 0.0;
    vehicle_state.orientation.x = 0.0;
    vehicle_state.orientation.y = 0.0;
    vehicle_state.orientation.z = sin (pose.Theta() / 2);
    vehicle_state.orientation.w = cos (pose.Theta() / 2);
    vehicle_state.velocity.x = 0.0; // to do
    vehicle_state.velocity.y = 0.0; // to do
    vehicle_state.velocity.z = 0.0;
    vehicle_state.rot_rate.x = 0.0;
    vehicle_state.rot_rate.y = 0.0;
    vehicle_state.rot_rate.z = 0.0; // to do
    // ignoring acc_bias, gyro_bias, and gravity
    vehicle_state_pub_.publish (vehicle_state);
  }
  
  
private:
  double width_;
  double length_;
  double align_distance_;
  double align_heading_;
  double vrot_;
  double vtrans_;
  std::string trajectory_topic_;
  std::string vehicle_state_topic_;
  size_t msg_queue_size_;
  
  npm::RobotServer * server_;
  boost::shared_ptr <npm::HoloDrive> drive_;
  ros::Subscriber trajectory_sub_;
  ros::Publisher vehicle_state_pub_;
  ros::Publisher trajectory_status_pub_;
  std::vector <TrajectoryPoint> trajectory_;
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
  
  npm::Factory::Instance().declare<CargoANTsMockup>("CargoANTsMockup");
  
  return 0;
}
