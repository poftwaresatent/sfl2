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

#include "ROSHAL.hpp"
#include <sfl/util/Pthread.hpp>
#include <sfl/util/numeric.hpp>

#include "ros/node.h"
#include "std_msgs/BaseVel.h"
#include "std_msgs/LaserScan.h"
#include "std_msgs/RobotBase2DOdom.h"

#include <err.h>
#include <sys/time.h>
#include <time.h>

using namespace sfl;
using namespace boost;
using namespace std;


#warning "Hardcoded Robox diff drive parameters"
static double const wheelradius(0.088);
static double const wheelbase(0.521);


namespace {
  
  class ROSHALNode
    : public ros::node
  {
  public:
    ROSHALNode(size_t nscanners, ostream * dbgos);
    ~ROSHALNode();
    
    void cbLaserScan(void * _channel);
    void cbRobotBase2DOdom();
    void publishSpeed();    
    
    shared_ptr<RWlock> rwlock;
    vector<uint32_t> rho_len;
    vector<double *> rho;
    double posx, posy, posth;
    double velx, velw;
    
  private:
    std_msgs::BaseVel m_vel;
    vector<std_msgs::LaserScan> m_scan;
    vector<string> m_scan_ad;
    vector<int> m_channel;
    std_msgs::RobotBase2DOdom m_odom;
    ostream * m_dbgos;
    vector<uint32_t> m_rho_alloc;
  };
  
}


ROSHAL::
ROSHAL(size_t nscanners, ostream * dbgos)
  : m_node(new ROSHALNode(nscanners, dbgos)),
    m_dbgos(dbgos)
{
}


ROSHAL::
~ROSHAL()
{
  delete m_node;
}


int ROSHAL::
time_get(timespec_t * stamp)
{
  struct timeval tv;
  int res(gettimeofday(&tv, 0));
  if(res != 0)
    return -1;
  TIMEVAL_TO_TIMESPEC(&tv, stamp);
  return 0;
}

  
int ROSHAL::
odometry_set(double x, double y, double theta,
	     double sxx, double syy, double stt,
	     double sxy, double sxt, double syt)
{
  if (m_dbgos)
    *m_dbgos << "ROSHAL::odometry_set(): ignoring your data, not connected to localization\n";
  return 0;
}


int ROSHAL::
odometry_get(timespec_t * stamp,
	     double * x, double * y, double * theta,
	     double * sxx, double * syy, double * stt,
	     double * sxy, double * sxt, double * syt)
{
#warning "Should take timestamp when the message is received in ROSHALNode."
  int res(time_get(stamp));
  if(res != 0)
    return res;
  
  *sxx = 1;
  *syy = 1;
  *stt = 1;
  *sxy = 0;
  *sxt = 0;
  *syt = 0;
  
  RWlock::rdsentry rd(m_node->rwlock);
  *x = m_node->posx;
  *y = m_node->posy;
  *theta = m_node->posth;
  
  return 0;
}


int ROSHAL::
speed_set(const double * qdot,
	  size_t * qdot_len)
{
  if (2 != *qdot_len)
    errx(EXIT_FAILURE, "ROSHAL::speed_set(): Sorry, I'm hardcoded for diff drives");
  
  double const dl(qdot[0] * wheelradius);
  double const dr(qdot[1] * wheelradius);
  
  {
    RWlock::wrsentry wr(m_node->rwlock);
    m_node->velx = (dl + dr) / 2;
    m_node->velw = (dr - dl) / wheelbase;
  }
  
  m_node->publishSpeed();
  
  return 0;
}


int ROSHAL::
speed_get(double * qdot,
	  size_t * qdot_len)
{
  if (2 != *qdot_len)
    errx(EXIT_FAILURE, "ROSHAL::speed_get(): Sorry, I'm hardcoded for diff drives");
  
  RWlock::rdsentry rd(m_node->rwlock);
  double const foo(m_node->velw * 0.5 * wheelbase);
  qdot[1] = (m_node->velx + foo) / wheelradius;
  qdot[0] = (m_node->velx - foo) / wheelradius;
  
  return 0;
}


int ROSHAL::
scan_get(int channel, double * rho,
	 size_t * rho_len,
	 timespec_t * t0, timespec_t * t1)
{
  if ((0 > channel) || (m_node->rho.size() <= static_cast<size_t>(channel)))
    errx(EXIT_FAILURE, "ROSHAL::scan_get(): invalid channel %d (have %zu scanners)",
	 channel, m_node->rho.size());
  
#warning "Should take timestamp when the message is received in ROSHALNode."
  int res(time_get(t0));
  if(res != 0)
    return res;
  *t1 = *t0;	      // should keep track of old and new timestamp...
  
  RWlock::rdsentry rd(m_node->rwlock);
  *rho_len = minval(*rho_len, m_node->rho_len[channel]);
  for (size_t is(0); is < *rho_len; ++is)
    rho[is] = m_node->rho[channel][is];
  return 0;
}


namespace {
  
  
  ROSHALNode::
  ROSHALNode(size_t nscanners, ostream * dbgos)
    : ros::node("ROSHALNode"),
      rwlock(RWlock::Create("ROSHALNode")),
      m_dbgos(dbgos)
  {
    if ( ! rwlock)
      errx(EXIT_FAILURE, "RWlock::Create(ROSHALNode) failed");
      
    for (size_t ii(0); ii < nscanners; ++ii) {
      ostringstream os;
      os << "scan" << ii;
      
      rho_len.push_back(0);
      rho.push_back(0);
      m_scan.push_back(std_msgs::LaserScan());
      m_scan_ad.push_back(os.str());
      m_channel.push_back(ii);
      m_rho_alloc.push_back(0);
      
      static int const max_queue(1);
      if ( ! subscribe(m_scan_ad[ii], m_scan[ii], &ROSHALNode::cbLaserScan,
		       &m_channel[ii], max_queue))
	errx(EXIT_FAILURE, "ROSHALNode: subscribe(%s) failed", m_scan_ad[ii].c_str());
      if (m_dbgos)
	*m_dbgos << "ROSHALNode: subscribed to " << m_scan_ad[ii] << "\n";
    }
    
    if ( ! subscribe("odom", m_odom, &ROSHALNode::cbRobotBase2DOdom, 1))
      errx(EXIT_FAILURE, "ROSHALNode: subscribe(odom) failed");
    if (m_dbgos)
      *m_dbgos << "ROSHALNode: subscribed to odom\n";
    
    if ( ! advertise<std_msgs::BaseVel>("cmd_vel", 1))
      errx(EXIT_FAILURE, "ROSHALNode: advertise(cmd_vel) failed");
    if (m_dbgos)
      *m_dbgos << "ROSHALNode: advertised cmd_vel\n";
  }
  
  
  ROSHALNode::
  ~ROSHALNode()
  {
    for (size_t ii(0); ii < m_scan.size(); ++ii)
      free(rho[ii]);
  }
  
  
  void ROSHALNode::
  cbLaserScan(void * _channel)
  {
    int const channel(*reinterpret_cast<int*>(_channel));
    if ((0 > channel) || (m_scan.size() <= static_cast<size_t>(channel)))
      errx(EXIT_FAILURE, "ROSHAL: invalid channel %d (I have %zu scanners)",
	   channel, m_scan.size());
    
    if (m_dbgos)
      *m_dbgos << "ROSHAL: LaserScan: " << channel << " " << m_scan[channel].angle_min << " "
	       << m_scan[channel].angle_max << " " << m_scan[channel].angle_increment << " "
	       << m_scan[channel].time_increment << " " << m_scan[channel].scan_time << " "
	       << m_scan[channel].range_min << " " << m_scan[channel].range_max << " "
	       << m_scan[channel].ranges_size << "\n";
    
    RWlock::wrsentry wr(rwlock);
    
    // check for change in scan size (should only happen at first message though)
    if (rho_len[channel] != m_scan[channel].ranges_size) {
      
      // grow allocated region, if necessary
      if (m_rho_alloc[channel] < m_scan[channel].ranges_size) {
	double * tmp;
	if (0 == rho[channel])
	  tmp = reinterpret_cast<double *>(calloc(m_scan[channel].ranges_size, sizeof(double)));
	else
	  tmp = reinterpret_cast<double *>(realloc(rho[channel],
						   m_scan[channel].ranges_size
						   * sizeof(double)));
	if (0 == tmp)
	  err(EXIT_FAILURE, "ROSHAL: calloc() or realloc() failed");
	rho[channel] = tmp;
	m_rho_alloc[channel] = m_scan[channel].ranges_size;
      }
	
      // record new length (might be smaller than allocated region)
      rho_len[channel] = m_scan[channel].ranges_size;
    }
    
    // copy ranges over into local buffer for use by ROSHAL
    for (uint32_t ii(0); ii < m_scan[channel].ranges_size; ++ii)
      rho[channel][ii] = m_scan[channel].ranges[ii];
  }
  
  
  void ROSHALNode::
  cbRobotBase2DOdom()
  {    
    if (m_dbgos)
      *m_dbgos << "ROSHAL: RobotBase2DOdom: " << m_odom.pos.x << " " << m_odom.pos.y << " "
	       << m_odom.pos.th << " " << m_odom.vel.x << " " << m_odom.vel.y << " "
	       << m_odom.vel.th << " " << static_cast<unsigned int>(m_odom.stall) << "\n";
    RWlock::wrsentry wr(rwlock);
    posx = m_odom.pos.x;
    posy = m_odom.pos.y;
    posth = m_odom.pos.th;
  }
  
  
  void ROSHALNode::
  publishSpeed()
  {
    {
      RWlock::rdsentry rd(rwlock);
      m_vel.vx = velx;
      m_vel.vw = velw;
    }
    publish("cmd_vel", m_vel);
    if (m_dbgos)
      *m_dbgos << "ROSHAL: published " << m_vel.vx << "  " << m_vel.vw << "\n";
  }
  
}
