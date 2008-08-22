/*
 * Copyright (c) 2008 Roland Philippsen <roland DOT philippsen AT gmx DOT net>
 *
 * BSD license:
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holder nor the names of
 *    contributors to this software may be used to endorse or promote
 *    products derived from this software without specific prior written
 *    permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHORS AND CONTRIBUTORS ``AS IS''
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED
 * TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
 * PARTICULAR PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT
 * HOLDER OR THE CONTRIBUTORS TO THIS SOFTWARE BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include "ROSbot.hpp"
#include "../common/OdometryDrawing.hpp"
#include "../common/HAL.hpp"
#include "../common/RobotServer.hpp"
#include "../common/Lidar.hpp"
#include "../common/DiffDrive.hpp"
#include "../common/RobotDescriptor.hpp"
#include <sfl/api/Goal.hpp>
#include <sfl/api/RobotModel.hpp>
#include <sfl/api/MotionController.hpp>
#include <sfl/util/Polygon.hpp>
#include <sfl/expo/expo_parameters.h>
#include <iostream>
#include <err.h>

#include "ros/node.h"
#include "std_msgs/BaseVel.h"
#include "std_msgs/LaserScan.h"
#include "std_msgs/RobotBase2DOdom.h"

// just for updating ../common/ScannerDrawing???
#include <sfl/api/Scanner.hpp>


using namespace npm;
using namespace sfl;
using namespace boost;
using namespace std;


class ROSbotNode : public ros::node {
public:
  ROSbot * bot;
  std_msgs::BaseVel vel;
  shared_ptr<RWlock> vel_rwlock;
  vector<std_msgs::LaserScan> scan;
  vector<string> scan_ad;
  std_msgs::RobotBase2DOdom odom;
    
  explicit ROSbotNode(ROSbot * _bot)
    : ros::node("ROSbotNode"),
      bot(_bot),
      vel_rwlock(RWlock::Create("ROSbotNode"))
  {
    if ( ! vel_rwlock)
      errx(EXIT_FAILURE, "RWlock::Create(ROSbotNode) failed");
      
    advertise<std_msgs::RobotBase2DOdom>("odom");
      
    vel.vx = 0;
    vel.vw = 0;
    if ( ! subscribe("cmd_vel", vel, &ROSbotNode::cbBaseVel))
      errx(EXIT_FAILURE, "subscribe(BaseVel) failed");
  }
    
  void PublishScans() {
    // lazy init so we don't worry about construction-time order
    if (scan_ad.empty()) {
      if (bot->m_lidar.empty())
	return;
      for (size_t ii(0); ii < bot->m_lidar.size(); ++ii) {
	Scanner const & scanner(*bot->m_lidar[ii]->GetScanner());
	  
	scan.push_back(std_msgs::LaserScan());
	scan[ii].set_ranges_size(scanner.nscans);
	scan[ii].angle_min = scanner.phi0;
	scan[ii].angle_max = scanner.phi0 + scanner.phirange;
	scan[ii].angle_increment = scanner.dphi;
	scan[ii].time_increment = 0.1 / scanner.nscans; // well... hardcoded to 10Hz
	scan[ii].scan_time = 0.1;
	scan[ii].range_min = 0;
	scan[ii].range_max = scanner.rhomax;

	ostringstream os;
	os << "scan" << ii;
	scan_ad.push_back(os.str());
	advertise<std_msgs::LaserScan>(os.str());
	  
	cout << "advertised " << os.str() << "\n";
      }
    }
      
    for (size_t ii(0); ii < scan.size(); ++ii) {
      Scanner const & scanner(*bot->m_lidar[ii]->GetScanner());
      for (size_t jj(0); jj < scanner.nscans; ++jj) {
	double rho;
	if (Scanner::INDEX_ERROR == scanner.Rho(ii, rho))
	  errx(EXIT_FAILURE, "bug? Scanner::INDEX_ERROR in Scanner %zu index %zu", ii, jj);
	scan[ii].ranges[jj] = rho;
      }
      publish(scan_ad[ii], scan[ii]);
      cout << "published " << scan_ad[ii] << "\n";
    }
  }
    
  void PublishOdom(bool stalled) {
    double xx, yy, th;
    bot->GetPose(xx, yy, th);
    odom.pos.x = xx;
    odom.pos.y = yy;
    odom.pos.th = th;
    if ( ! bot->m_drive->ComputeSpeedState(xx, yy, th))
      errx(EXIT_FAILURE, "bot->m_drive->ComputeSpeedState() failed");	// a bit harsh maybe
    odom.vel.x = xx;
    odom.vel.y = yy;
    odom.vel.th = th;
    odom.stall = stalled ? 1 : 0;
    publish("odom", odom);
    cout << "published odom\n";
  }
    
  void cbBaseVel() {
    RWlock::wrsentry wrs(vel_rwlock);
    cout << "RECEIVED vx=" << vel.vx << " vw=" << vel.vw << "\n";
  }
};


ROSbot::
ROSbot(shared_ptr<RobotDescriptor> descriptor, const World & world)
  : RobotClient(descriptor, world, 2, true),
    m_goal(new Goal()),
    m_hull(new Hull())
{
  {
    static double const octoSmall = 0.178;
    static double const octoBig = 0.430;
    
    Polygon outline;
    outline.AddPoint( octoBig  , octoSmall);
    outline.AddPoint( octoSmall, octoBig);
    outline.AddPoint(-octoSmall, octoBig);
    outline.AddPoint(-octoBig  , octoSmall);
    outline.AddPoint(-octoBig  ,-octoSmall);
    outline.AddPoint(-octoSmall,-octoBig);
    outline.AddPoint( octoSmall,-octoBig);
    outline.AddPoint( octoBig  ,-octoSmall);
    
    AddPolygon(outline);
    m_hull->AddPolygon(outline);
  }
  
  // find a more generic way (maybe ROS services or parameters)
  expo_parameters params(descriptor);
  m_lidar.push_back(DefineLidar(Frame(params.front_mount_x,
				      params.front_mount_y,
				      params.front_mount_theta),
				params.front_nscans,
				params.front_rhomax,
				params.front_phi0,
				params.front_phirange,
				params.front_channel));
  m_lidar.push_back(DefineLidar(Frame(params.rear_mount_x,
				      params.rear_mount_y,
				      params.rear_mount_theta),
				params.rear_nscans,
				params.rear_rhomax,
				params.rear_phi0,
				params.rear_phirange,
				params.rear_channel));
  m_drive = DefineDiffDrive(params.model_wheelbase, params.model_wheelradius);
  
  RobotModel::Parameters const modelParms(params.model_security_distance,
 					  params.model_wheelbase,
 					  params.model_wheelradius,
 					  params.model_qd_max,
					  params.model_qdd_max,
					  params.model_sd_max,
					  params.model_thetad_max,
					  params.model_sdd_max,
					  params.model_thetadd_max);
  m_model.reset(new RobotModel(modelParms, m_hull));
  
  m_mcontrol.reset(new MotionController(m_model, GetHAL(), RWlock::Create("mcontrol")));
  
  {
    int argc(1);
    char * argv[1];
    argv[0] = "ROSbot";
    ros::init(argc, argv);
  }
  m_ros_node = new ROSbotNode(this);
}


ROSbot::
~ROSbot()
{
  ros::fini();
  delete m_ros_node;
}


void ROSbot::
InitPose(double x,
	 double y,
	 double theta)
{
  cout << "should we do something smart in ROSbot::InitPose()?\n";
}


void ROSbot::
SetPose(double x,
	double y,
	double theta)
{
  cout << "should we do something smart in ROSbot::SetPose()?\n";
}


void ROSbot::
GetPose(double & x,
	double & y,
	double & theta)
{
  const Frame * pose(GetServer()->GetNoisyPose());
  if ( ! pose)
    pose = & GetServer()->GetTruePose();
  x = pose->X();
  y = pose->Y();
  theta = pose->Theta();
}


shared_ptr<const Goal> ROSbot::
GetGoal()
{
  return m_goal;
}


bool ROSbot::
GoalReached()
{
  const Frame * pose(GetServer()->GetNoisyPose());
  if ( ! pose)
    pose = & GetServer()->GetTruePose();
  return m_goal->Reached(*pose, true); // "true" means goForward
}


void ROSbot::
SetGoal(double timestep, const Goal & goal)
{
  cout << "should we do something smart in ROSbot::SetGoal()?\n";
  *m_goal = goal;
}


bool ROSbot::
PrepareAction(double timestep)
{
  for (size_t ii(0); ii < m_lidar.size(); ++ii)
    m_lidar[ii]->GetScanner()->Update(); // just for drawing???
  
  m_ros_node->PublishScans();
  
  bool stalled(false);
  m_ros_node->PublishOdom(stalled);
  
  RWlock::rdsentry rds(m_ros_node->vel_rwlock);
  m_mcontrol->ProposeSpeed(m_ros_node->vel.vx, m_ros_node->vel.vw);
  return (0 == m_mcontrol->Update(timestep, &cout));
}
