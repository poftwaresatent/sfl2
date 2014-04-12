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
#include <npm/World.hpp>
#include <npm/Object.hpp>
#include <npm/gfx/TraversabilityDrawing.hpp>
#include <npm/gfx/wrap_glu.hpp>
#include <sfl/api/Pose.hpp>
#include <sfl/util/Line.hpp>
#include <sfl/util/numeric.hpp>
#include <iostream>
#include <cmath>

#include "ros/ros.h"
#include "cargo_ants_msgs/VehicleState.h"
#include "cargo_ants_msgs/Trajectory.h"
#include "cargo_ants_msgs/MockupMap.h"
#include "cargo_ants_msgs/Route.h"
#include "cargo_ants_msgs/Path.h"

// Not so happy about the way we end up needing to include generated
// message headers... too easily confused with
// i.e. <sfl/gplan/TraversabilityMap.hpp> maybe create a separate
// sfl_msgs package, then at least th einclude directive clearly marks
// things as messages.
//
#include "sfl2/TraversabilityMap.h"
#include "sfl2/Estar.h"

using namespace cargo_ants_msgs;


class TravmapMsgProxy
  : public npm::TravProxyAPI
{
public:
  TravmapMsgProxy ()
    : gframe_ (0, 0, 0, 1)
  {
  }
  
  void update (sfl2::TraversabilityMap::ConstPtr msg)
  {
    msg_ = msg;    
    gframe_.Configure (msg->gx, msg->gy, msg->gth, msg->delta);
  }
  
  virtual bool Enabled() const    { return msg_; }
  virtual double GetX() const     { return gframe_.X(); }
  virtual double GetY() const     { return gframe_.Y(); }
  virtual double GetTheta() const { return gframe_.Theta(); }
  virtual double GetDelta() const { return gframe_.Delta(); }
  virtual sfl::GridFrame const * GetGridFrame() { return &gframe_; }
  virtual int GetObstacle() const   { return msg_->obstacle; }
  virtual int GetFreespace() const  { return msg_->freespace; }
  virtual ssize_t GetXBegin() const { return msg_->xbegin; }
  virtual ssize_t GetXEnd() const   { return msg_->xend; }
  virtual ssize_t GetYBegin() const { return msg_->ybegin; }
  virtual ssize_t GetYEnd() const   { return msg_->yend; }
  
  virtual int GetValue(ssize_t ix, ssize_t iy) const {
    ssize_t const ii (ix - msg_->xbegin + (iy - msg_->ybegin) * (msg_->xend - msg_->xbegin));
    if (ii >= msg_->grid.size()) {
      return msg_->freespace - 1;
    }
    return msg_->grid[ii];
  }
  
private:
  sfl2::TraversabilityMap::ConstPtr msg_;
  
  sfl::GridFrame gframe_;
  int freespace_;
  int obstacle_;
  typedef sfl::flexgrid <int> grid_t;
  grid_t grid_;
};


class RouteDrawing
  : public npm::Drawing
{
public:
  explicit RouteDrawing (std::string const & name)
    : npm::Drawing (name, "draws route messages in the color of their vehicle")
  {
  }
  
  virtual void Draw()
  {
    for (route_t::const_iterator ir (route_.begin()); ir != route_.end(); ++ir) {
      npm::RobotClient const * robot (npm::RobotClient::registry.find (ir->first));
      npm::color_s cc (0.5, 0.5, 0.5);
      if (robot) {
	cc = robot->GetColor();
      }
      
      std::vector<cargo_ants_msgs::Goal> const & goals (ir->second->goals);
      
      for (size_t ig (0); ig < goals.size(); ++ig) {
      	glMatrixMode (GL_MODELVIEW);
      	glPushMatrix ();
      	glTranslated (goals[ig].gx, goals[ig].gy, 0);
	glColor3d (0.8 * cc.red, 0.8 * cc.green, 0.8 * cc.blue);
	gluDisk (wrap_glu_quadric_instance(), 0.0, goals[ig].dr, 36, 1);
      	glMatrixMode(GL_MODELVIEW);
      	glPopMatrix ();
      }
      
      start_t::const_iterator is (start_.find (ir->first));
      if (start_.end() != is) {
      	if (goals.size() > 0) {
	  glLineWidth (3);
	  glColor3d (0.8 * cc.red, 0.8 * cc.green, 0.8 * cc.blue);
      	  glBegin (GL_LINES);
      	  glVertex2d (is->second.X(), is->second.Y());
      	  for (size_t ig (0); ig < goals.size(); ++ig) {
	    if (ig > 0) {
	      glVertex2d (goals[ig-1].gx, goals[ig-1].gy);
	    }
      	    glVertex2d (goals[ig].gx, goals[ig].gy);
      	  }
      	  glEnd ();
      	}
      }
      else if (goals.size() > 1) {
      	glLineWidth (3);
	glColor3d (0.8 * cc.red, 0.8 * cc.green, 0.8 * cc.blue);
	glBegin (GL_LINES);
      	for (size_t ig (0); ig < goals.size(); ++ig) {
	  if (ig > 0) {
	    glVertex2d (goals[ig-1].gx, goals[ig-1].gy);
	  }
      	  glVertex2d (goals[ig].gx, goals[ig].gy);
      	}
      	glEnd ();
      }
    }
  }
  
  void update (Route::ConstPtr msg)
  {
    route_[msg->vehicle] = msg;
    npm::RobotClient const * robot (npm::RobotClient::registry.find (msg->vehicle));
    if (robot) {
      sfl::Pose pose;
      if (robot->GetPose (pose)) {
	start_[msg->vehicle] = pose;
      }
      else {
	start_.erase (msg->vehicle);
      }
    }
  }
  
private:
  typedef std::map <std::string, Route::ConstPtr> route_t;
  route_t route_;
  typedef std::map <std::string, sfl::Pose> start_t;
  start_t start_;
};


class PathDrawing
  : public npm::Drawing
{
public:
  PathDrawing (std::string const & name, npm::RobotClient const * robot)
    : npm::Drawing (name, "draws path messages"),
      robot_ (robot)
  {
  }
  
  virtual void Draw()
  {
    if ( ! path_) {
      return;
    }
    
    npm::color_s const & cc (robot_->GetColor());
    glColor3d (0.4 * cc.red, 0.4 * cc.green, 0.4 * cc.blue);
    
    std::vector<cargo_ants_msgs::Goal> const & goals (path_->goals);
    
    glPolygonMode (GL_FRONT, GL_FILL);
    for (size_t ig (0); ig < goals.size(); ++ig) {
      glMatrixMode (GL_MODELVIEW);
      glPushMatrix ();
      glTranslated (goals[ig].gx, goals[ig].gy, 0);
      gluDisk (wrap_glu_quadric_instance(), 0.0, goals[ig].dr, 36, 1);
      glMatrixMode(GL_MODELVIEW); // ???
      glPopMatrix ();
    }
    
    glPointSize (3);
    glBegin (GL_POINTS);
    for (size_t ig (0); ig < goals.size(); ++ig) {
      glVertex2d (goals[ig].gx, goals[ig].gy);
    }
    glEnd ();
  }
  
  void update (Path::ConstPtr msg)
  {
    path_ = msg;
  }
  
private:
  npm::RobotClient const * robot_;
  Path::ConstPtr path_;
};


class EstarDrawing
  : public npm::Drawing
{
public:
  EstarDrawing (std::string const & name)
    : npm::Drawing (name, "E*")
  {
  }

  virtual void Draw()
  {
    if ( ! msg_) {
      return;
    }
    
    glMatrixMode (GL_MODELVIEW);
    glPushMatrix ();
    glTranslated (msg_->gx, msg_->gy, 0.0);
    glRotated (180.0 * msg_->gth / M_PI, 0.0, 0.0, 1.0);
    glScaled(msg_->delta, msg_->delta, 1.0);
    
    double delta (1.0);
    for (size_t ii (0); ii < msg_->value.size(); ++ii) {
      if (msg_->value[ii] <= 50 && msg_->value[ii] > delta) { // XXXX magic number
	delta = msg_->value[ii];
      }
    }
    
    glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
    for (size_t ix (0); ix < msg_->xdim; ++ix) {
      for (size_t iy (0); iy < msg_->ydim; ++iy) {
	size_t const ii (ix + msg_->xdim * iy);
	double const vv (fmin (msg_->value[ii], delta) / delta);
	if (vv < 0) {
	  glColor3d (0, 0, 1);
	}
	else if (vv < 1e-3) {	// XXXX magic number
	  glColor3d(0, 1, 0);
	}
	else if (std::isinf (vv)) {
	  glColor3d(1, 0, 0);
	}
	else {
	  glColor3d (vv, vv, vv);
	}
	glRectd(ix - 0.5, iy - 0.5, ix + 0.5, iy + 0.5);
      }
    }
    
    glMatrixMode(GL_MODELVIEW);
    glPopMatrix();
  }
  
  void update (sfl2::Estar::ConstPtr msg)
  {
    ROS_INFO ("EstarDrawing gframe: %g  %g  %g  %g  dim: %lldx%lld", msg->gx, msg->gy, msg->gth, msg->delta, msg->xdim, msg->ydim);
    msg_ = msg;
  }
  
private:
  sfl2::Estar::ConstPtr msg_;
};


class MockupBase
  : public npm::RobotClient	// it's not really a robot, but easiest to hack in this way
{
public:
  MockupBase (std::string const & name)
    : npm::RobotClient (name),
      msg_queue_size_ (10),
      server_ (0)
  {
    reflectParameter ("msg_queue_size", &msg_queue_size_);
  }
  
  
  virtual bool Initialize (npm::RobotServer & server)
  {
    if ( ! npm::RobotClient::Initialize(server)) {
      return false;
    }
    server_ = &server;
    return init();
  }
  
  
  virtual bool PrepareAction (double timestep)
  {
    if ( ! ros::ok()) {
      return false;
    }
    ros::spinOnce();
    return update (timestep);
  }
  
  
  virtual void InitPose (sfl::Pose const & pose) {}
  virtual void SetPose (sfl::Pose const & pose) {}
  
  virtual bool GetPose (sfl::Pose & pose) const
  {
    if ( ! server_) {
      return false;
    }
    pose = server_->GetTruePose();
    return true;
  }
  
  virtual void SetGoal (double timestep, const sfl::Goal & goal) {}
  virtual bool GetGoal (sfl::Goal & goal) const { return false; }
  virtual bool GoalReached () const { return false; }
  
  virtual bool init () = 0;
  virtual bool update (double timestep) = 0;
  
protected:
  size_t msg_queue_size_;
  npm::RobotServer * server_;
};


class MockupGlue
  : public MockupBase
{
public:
  MockupGlue (std::string const & name)
    : MockupBase (name),
      site_map_topic_ ("site_map"),
      route_topic_ ("route")
  {
    reflectParameter ("site_map_topic", &site_map_topic_);
    reflectParameter ("route_topic", &route_topic_);
  }
  
  virtual bool init ()
  {
    ros::NodeHandle node;
    site_map_pub_ = node.advertise <MockupMap> (site_map_topic_, 1);
    route_sub_ = node.subscribe (route_topic_, msg_queue_size_, &MockupGlue::routeCB, this);
    route_drawing_.reset (new RouteDrawing(name + "_route_drawing"));
    return true;
  }
  
  void routeCB (Route::ConstPtr msg)
  {
    route_drawing_->update (msg);
  }
  
  virtual bool update (double timestep)
  {
    MockupMap site_map;
    MockupMapEntry entry;
    
    entry.name = "site";
    npm::World::object_t const & objects (server_->GetWorld().GetObjects());
    for (size_t io(0); io < objects.size(); ++io) {
      for (size_t il(0); il < objects[io]->GetNlines(); ++il) {
	sfl::Line const & ln (*objects[io]->GetGlobalLine(il));
	entry.x0.push_back (ln.X0());
	entry.y0.push_back (ln.Y0());
	entry.x1.push_back (ln.X1());
	entry.y1.push_back (ln.Y1());
      }
    }
    site_map.entries.push_back (entry);
    
    npm::World::robot_t const & robots (server_->GetWorld().GetRobots());
    for (size_t ir(0); ir < robots.size(); ++ir) {
      if (robots[ir]->GetClient() == this) {
	// Quick hack because MockupGlue is a RobotClient. Nepumuk
	// needs a little refactoring to make implementing this
	// easier.
	continue;
      }
      entry.name = robots[ir]->GetName();
      entry.x0.clear();
      entry.y0.clear();
      entry.x1.clear();
      entry.y1.clear();
      npm::Object const & body (robots[ir]->GetBody());
      for (size_t il(0); il < body.GetNlines(); ++il) {
	sfl::Line const & ln (*body.GetGlobalLine(il));
	entry.x0.push_back (ln.X0());
	entry.y0.push_back (ln.Y0());
	entry.x1.push_back (ln.X1());
	entry.y1.push_back (ln.Y1());
      }
      site_map.entries.push_back (entry);
    }
    
    site_map_pub_.publish (site_map);
    return true;
  }
  
private:
  std::string site_map_topic_;
  std::string route_topic_;
  ros::Publisher site_map_pub_;
  ros::Subscriber route_sub_;
  boost::shared_ptr<RouteDrawing> route_drawing_;
};


class MockupRobot
  : public MockupBase
{
public:
  explicit MockupRobot (std::string const & name)
    : MockupBase (name),
      width_ (2.0),
      length_ (4.0),
      align_distance_ (1.5),
      align_heading_ (0.3),
      vrot_ (45 * M_PI / 180),
      vtrans_ (2.0),
      trajectory_topic_ ("trajectory"),
      vehicle_state_topic_ ("vehicle_state"),
      travmap_topic_ ("travmap"),
      travmap_proxy_ (new TravmapMsgProxy()),
      estar_topic_("estar"),
      path_topic_ ("path")
  {
    reflectParameter ("width", &width_);
    reflectParameter ("length", &length_);
    reflectParameter ("align_distance", &align_distance_);
    reflectParameter ("align_heading", &align_heading_);
    reflectParameter ("vrot", &vrot_);
    reflectParameter ("vtrans", &vtrans_);
    reflectParameter ("trajectory_topic", &trajectory_topic_);
    reflectParameter ("vehicle_state_topic", &vehicle_state_topic_);
    reflectParameter ("travmap_topic", &travmap_topic_);
    reflectParameter ("estar_topic", &estar_topic_);
    reflectParameter ("path_topic", &path_topic_);
  }
  
  
  virtual bool init ()
  {
    drive_ = server_->DefineHoloDrive(0.8 * width_);
    
    server_->AddLine (-length_/2, -width_/2, -length_/2,  width_/2);
    server_->AddLine (-length_/2,  width_/2,  length_/2,  width_/2);
    server_->AddLine ( length_/2,  width_/2,  length_/2, -width_/2);
    server_->AddLine ( length_/2, -width_/2, -length_/2, -width_/2);
    
    ros::NodeHandle node;
    trajectory_sub_ = node.subscribe (trajectory_topic_, msg_queue_size_,
				      &MockupRobot::trajectoryCB, this);
    vehicle_state_pub_ = node.advertise <VehicleState> (vehicle_state_topic_, 1);
    travmap_sub_ = node.subscribe (travmap_topic_, msg_queue_size_,
				   &MockupRobot::travmapCB, this);
    estar_sub_ = node.subscribe (estar_topic_, msg_queue_size_,
				 &MockupRobot::estarCB, this);
    path_sub_ = node.subscribe (path_topic_, msg_queue_size_,
				&MockupRobot::pathCB, this);
    
    travmap_drawing_.reset (new npm::TraversabilityDrawing (name + "_travmap", travmap_proxy_));
    travmap_camera_.reset (new npm::TraversabilityCamera (name + "_travmap", travmap_proxy_));
    estar_drawing_.reset (new EstarDrawing (name + "_estar"));
    path_drawing_.reset (new PathDrawing (name + "_path", this));
    
    return true;
  }
  
  
  virtual bool update (double timestep)
  {
    if ( ! trajectory_.empty()) {
      if ( ! move (trajectory_.back(), timestep)) {
	return false;
      }
    }
    publish ();
    return true;
  }
  
  
  void trajectoryCB (Trajectory::ConstPtr const & msg)
  {
    trajectory_ = msg->points;
  }
  
  
  void travmapCB (sfl2::TraversabilityMap::ConstPtr const & msg)
  {
    travmap_proxy_->update (msg);
  }
  
  
  void estarCB (sfl2::Estar::ConstPtr const & msg)
  {
    estar_drawing_->update (msg);
  }
  
  
  void pathCB (cargo_ants_msgs::Path::ConstPtr const & msg)
  {
    path_drawing_->update (msg);
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
      }
      else {
	qd[0] = sfl::boundval (-vtrans_, dist  / timestep, vtrans_);
	qd[1] = 0.0;
	qd[2] = sfl::boundval (-vrot_,   dhead / timestep, vrot_);
      }
    }
    else {
      qd[0] = sfl::boundval (-vtrans_, target_pose.X()     / timestep, vtrans_);
      qd[1] = sfl::boundval (-vtrans_, target_pose.Y()     / timestep, vtrans_);
      qd[2] = sfl::boundval (-vrot_,   target_pose.Theta() / timestep, vrot_);
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
  std::string travmap_topic_;
  std::string estar_topic_;
  std::string path_topic_;
  
  boost::shared_ptr <npm::HoloDrive> drive_;
  ros::Subscriber trajectory_sub_;
  ros::Subscriber travmap_sub_;
  ros::Subscriber estar_sub_;
  ros::Subscriber path_sub_;
  ros::Publisher vehicle_state_pub_;
  ros::Publisher trajectory_status_pub_;
  std::vector <TrajectoryPoint> trajectory_;
  boost::shared_ptr <TravmapMsgProxy> travmap_proxy_;
  boost::shared_ptr <npm::TraversabilityDrawing> travmap_drawing_;
  boost::shared_ptr <npm::TraversabilityCamera> travmap_camera_;
  boost::shared_ptr <EstarDrawing> estar_drawing_;
  boost::shared_ptr <PathDrawing> path_drawing_;
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
  
  npm::Factory::Instance().declare<MockupGlue>("CargoANTsMockupGlue");
  npm::Factory::Instance().declare<MockupRobot>("CargoANTsMockupRobot");
  
  return 0;
}
