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

#include "ros/node.h"
#include "std_msgs/BaseVel.h"
#include "std_msgs/LaserScan.h"
#include "std_msgs/RobotBase2DOdom.h"
#include <err.h>
#include <signal.h>
#include <unistd.h>
#include <iostream>

using namespace std;


class Base : public ros::node {
public:
  std_msgs::BaseVel vel;
  std_msgs::LaserScan scan;
  std_msgs::RobotBase2DOdom odom;
  
  explicit Base(string const & name);
  
  void cbBaseVel();  
  void cbLaserScan();
  void cbRobotBase2DOdom();
  
  virtual void Update() = 0;
  virtual void Stop() = 0;
};


class Robot : public Base {
public:
  Robot();
  virtual void Update();
  virtual void Stop();
};


class Sim : public Base {
public:
  Sim();
  virtual void Update();
  virtual void Stop();
};


static void handle(int signum);
static void cleanup();
static void usage();
static void parse_options(int argc, char ** argv);

static bool keep_running(true);
static bool sim_mode(false);
static bool ros_intr(false);
static class Base * base;


int main(int argc, char **argv)
{
  parse_options(argc, argv);
  ros::init(argc, argv);
  if ( -1 == atexit(cleanup))
    err(EXIT_FAILURE, "atexit()");
  if (( ! ros_intr) && (SIG_ERR == signal(SIGINT, handle)))
    err(EXIT_FAILURE, "signal()");
  if (sim_mode)
    base = new Sim();
  else
    base = new Robot();
  while (base->ok() && keep_running) {
    base->Update();
    usleep(1000000);
  }
  return EXIT_SUCCESS;
}


static void handle(int signum)
{
  warnx("caught signal %d", signum);
  if ( ! keep_running) {
    ros::fini();
    errx(EXIT_FAILURE, "should have exited last time we caught that...");
  }
  keep_running = false;
}


static void cleanup()
{
  if (base)
    base->Stop();
  warnx("calling ros::fini()");
  ros::fini();
  warnx("destroying ROS nodes");
  delete base;
  warnx("if ROS claims we did not call ros::fini(), somebody is confused");
}


static void usage()
{
  warnx("command line options:\n"
	"  -h  this help message\n"
	"  -s  use simulator mode (send scans and odometry, receive base vel)\n"
	"  -i  let ROS handle SIGINT");
}


static void parse_options(int argc, char ** argv)
{
  char const * opts("hsi");
  while (true) {
    int opt(getopt(argc, argv, opts));
    if (-1 == opt)
      break;
    switch (opt) {
    case 'h':
      usage();
      exit(EXIT_SUCCESS);
    case 's':
      sim_mode = true;
      break;
    case 'i':
      ros_intr = true;
      break;
    case '?':
    default:
      usage();
      errx(EXIT_FAILURE, "unrecognized option %c", optopt);
    }
  }
}


Base::Base(string const & name)
  : ros::node(name, ros_intr ? 0 : ros::node::DONT_HANDLE_SIGINT)
{
}


void Base::cbBaseVel()
{
  cout << "BaseVel: " << vel.vx << " " << vel.vw << "\n";
}


void Base::cbLaserScan()
{
  cout << "LaserScan: " << scan.angle_min << " " << scan.angle_max << " "
       << scan.angle_increment << " " << scan.time_increment << " " << scan.scan_time << " "
       << scan.range_min << " " << scan.range_max << " " << scan.ranges_size << "\n";
  //     for (uint32_t ii(0); ii < scan.ranges_size; ++ii)
  //       cout << " " << scan.ranges[ii];
  //     cout << "\n";
}


void Base::cbRobotBase2DOdom()
{
  cout << "RobotBase2DOdom: " << odom.pos.x << " " << odom.pos.y << " "
       << odom.pos.th << " " << odom.vel.x << " " << odom.vel.y << " "
       << odom.vel.th << " " << static_cast<unsigned int>(odom.stall) << "\n";
}


Robot::Robot()
  : Base("robot")
{
  if ( ! subscribe("scan0", scan, &Base::cbLaserScan))
    errx(EXIT_FAILURE, "subscribe(LaserScan) failed");
  if ( ! subscribe("scan1", scan, &Base::cbLaserScan))
    errx(EXIT_FAILURE, "subscribe(LaserScan) failed");
  if ( ! subscribe("odom", odom, &Base::cbRobotBase2DOdom))
    errx(EXIT_FAILURE, "subscribe(RobotBase2DOdom) failed");
  
  advertise<std_msgs::BaseVel>("cmd_vel");
}


void Robot::Update()
{
  vel.vx = 0.1;
  vel.vw = 0.1;
  publish("cmd_vel", vel);
  cout << "published " << vel.vx << "  " << vel.vw << "\n";
}


void Robot::Stop()
{
  vel.vx = 0;
  vel.vw = 0;
  publish("cmd_vel", vel);
  cout << "bye bye Robot " << vel.vx << "  " << vel.vw << "\n";
}


Sim::Sim()
  : Base("sim")
{
  if ( ! subscribe("cmd_vel", vel, &Base::cbBaseVel))
    errx(EXIT_FAILURE, "subscribe(BaseVel) failed");
  
  advertise<std_msgs::LaserScan>("scan0");
  advertise<std_msgs::LaserScan>("scan1");
  advertise<std_msgs::RobotBase2DOdom>("odom");
}


void Sim::Update()
{
  if (scan.get_ranges_size() != 15) {
    scan.set_ranges_size(15);
    scan.angle_min = -1;
    scan.angle_max = +1;
    scan.angle_increment = 2.0 / 14;
    scan.time_increment = 0.1 / 15;
    scan.scan_time = 0.1;
    scan.range_min = 1.1;
    scan.range_max = 2.2;
    for (size_t jj(0); jj < 15; ++jj)
      scan.ranges[jj] = scan.range_min * ((14 - jj) / 14.0) + scan.range_max * (jj / 14.0);
    
    odom.pos.x = 0.0;
    odom.pos.y = 0.1;
    odom.pos.th = 0.2;
    odom.vel.x = 0.3;
    odom.vel.y = 0.4;
    odom.vel.th = 0.5;
    odom.stall = 0;
  }
  
  publish("scan0", scan);
  publish("scan1", scan);
  cout << "published scans " << scan.angle_min << " " << scan.angle_max << " "
       << scan.angle_increment << " " << scan.time_increment << " " << scan.scan_time << " "
       << scan.range_min << " " << scan.range_max << " " << scan.ranges_size << "\n";
  
  publish("odom", odom);
  cout << "published odom " << odom.pos.x << " " << odom.pos.y << " "
       << odom.pos.th << " " << odom.vel.x << " " << odom.vel.y << " "
       << odom.vel.th << " " << static_cast<unsigned int>(odom.stall) << "\n";
}


void Sim::Stop()
{
  cout << "bye bye Sim\n";
}
