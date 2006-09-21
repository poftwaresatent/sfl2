// g++ -o testcwrap -Wall -I/Users/rolo/local-sfl/include testcwrap.cpp -L/Users/rolo/local-sfl/lib -lsunflower

#include <sfl/cwrap/hal.h>
#include <sfl/cwrap/sfl.h>
#include <sfl/cwrap/expo.h>
#include <sfl/api/RobotModel.hpp>
#include <sfl/api/Goal.hpp>
#include <sfl/dwa/DynamicWindow.hpp>
#include <iostream>
#include <cmath>
#include <sys/time.h>


using namespace sfl;
using namespace sfl_cwrap;
using namespace boost;
using namespace std;


static const double LIDAROFFSET = 0.15;
static const double WHEELBASE = 0.521;
static const double WHEELRADIUS = 0.088;
static const int front_channel(0);
static const int rear_channel(1);
static const int nscans = 181;
static const double rhomax = 8;
static const double phi0 = - M_PI / 2;
static const double phirange = M_PI;
static const double octoSmall = 0.178;
static const double octoBig = 0.430;

static const RobotModel::Parameters
modelparms(0.02, // safety distance
	   WHEELBASE, // wheel base
	   WHEELRADIUS, // wheel radius
	   6.5, // qd max
	   6.5, // qdd max
	   0.5, // sd max
	   2.0, // thetad max
	   0.75 * WHEELRADIUS * 6.5, // sdd max
	   1.5 * WHEELRADIUS * 6.5 / WHEELBASE // thetadd max
	   );


static int hal_time_get(struct cwrap_hal_s * self, struct timespec * stamp);
static int hal_odometry_set(struct cwrap_hal_s * self,
			    double x, double y, double theta,
			    double sxx, double syy, double stt,
			    double sxy, double sxt, double syt);
static int hal_odometry_get(struct cwrap_hal_s * self, struct timespec * stamp,
			    double * x, double * y, double * theta,
			    double * sxx, double * syy, double * stt,
			    double * sxy, double * sxt, double * syt);
static int hal_speed_set(struct cwrap_hal_s * self, double qdl, double qdr);
static int hal_speed_get(struct cwrap_hal_s * self,
			 double * qdl, double * qdr);
static int hal_scan_get(struct cwrap_hal_s * self,
			int channel, double * rho, int rho_len,
			struct timespec * t0, struct timespec * t1);


struct hal_s {
  double x, y, theta, sxx, syy, stt, sxy, sxt, syt;
  double qdl, qdr;
};


class Robot
{
public:
  Robot();
  ~Robot();
  
  void PrepareAction(double timestep);
  void SetGoal(const Goal & goal);
  void DumpObstacles();
  
  struct cwrap_hal_s m_cwrap_hal;
  
  int m_hal_handle;
  int m_front_handle;
  int m_rear_handle;
  int m_robotModel_handle;
  int m_motionController_handle;
  int m_dynamicWindow_handle;
  int m_odometry_handle;
  int m_bubbleBand_handle;
  int m_multiscanner_handle;
  int m_motionPlanner_handle;
};


hal_s hal;


int main(int argc, char ** argv)
{
  Robot robot;
  robot.SetGoal(Goal(10, 4, 0, 0.1, M_PI / 4, false));
  robot.PrepareAction(0.1);
  robot.DumpObstacles();
}


Robot::
Robot()
{
  m_cwrap_hal.time_get     = hal_time_get;
  m_cwrap_hal.odometry_set = hal_odometry_set;
  m_cwrap_hal.odometry_get = hal_odometry_get;
  m_cwrap_hal.speed_set    = hal_speed_set;
  m_cwrap_hal.speed_get    = hal_speed_get;
  m_cwrap_hal.scan_get     = hal_scan_get;
  m_hal_handle = sfl_create_HAL(&m_cwrap_hal);
  if(0 > m_hal_handle){
    cerr << "sfl_create_HAL(): " << m_hal_handle << "\n";
    exit(EXIT_FAILURE);
  }

  m_front_handle =
    sfl_create_Scanner(m_hal_handle, front_channel,
		       LIDAROFFSET, 0, 0, nscans, rhomax, phi0, phirange);
  if(0 > m_front_handle){
    cerr << "sfl_create_Scanner(front): " <<  m_front_handle << "\n";
    exit(EXIT_FAILURE);
  }
  m_rear_handle =
    sfl_create_Scanner(m_hal_handle, rear_channel,
		       -LIDAROFFSET, 0, M_PI, nscans, rhomax, phi0, phirange);
  if(0 > m_rear_handle){
    cerr << "sfl_create_Scanner(rear): " <<  m_rear_handle << "\n";
    exit(EXIT_FAILURE);
  }
  
  double tmp_hull_x[8];
  double tmp_hull_y[8];
  tmp_hull_x[0] =  octoBig;   tmp_hull_y[0] =  octoSmall;
  tmp_hull_x[1] =  octoSmall; tmp_hull_y[1] =  octoBig;
  tmp_hull_x[2] = -octoSmall; tmp_hull_y[2] =  octoBig;
  tmp_hull_x[3] = -octoBig;   tmp_hull_y[3] =  octoSmall;
  tmp_hull_x[4] = -octoBig;   tmp_hull_y[4] = -octoSmall;
  tmp_hull_x[5] = -octoSmall; tmp_hull_y[5] = -octoBig;
  tmp_hull_x[6] =  octoSmall; tmp_hull_y[6] = -octoBig;
  tmp_hull_x[7] =  octoBig;   tmp_hull_y[7] = -octoSmall;
  m_robotModel_handle =
    sfl_create_RobotModel(modelparms.safetyDistance, modelparms.wheelBase,
			  modelparms.wheelRadius, modelparms.qdMax,
			  modelparms.qddMax, modelparms.sdMax,
			  modelparms.thetadMax, modelparms.sddMax,
			  modelparms.thetaddMax, tmp_hull_x, tmp_hull_y, 8);
  if(0 > m_robotModel_handle){
    cerr << "sfl_create_RobotModel(): " << m_robotModel_handle << "\n";
    exit(EXIT_FAILURE);
  }
  
  m_motionController_handle =
    expo_create_MotionController(m_robotModel_handle, m_hal_handle);
  if(0 > m_motionController_handle){
    cerr << "sfl_create_MotionController(): " << m_motionController_handle
	 << "\n";
    exit(EXIT_FAILURE);
  }
  
  m_dynamicWindow_handle =
    sfl_create_DynamicWindow(m_robotModel_handle,
			     m_motionController_handle,
			     21, // dimension
			     2.2, // grid width
			     1.5, // grid height
			     0.1, // grid resolution
			     0.5, // alpha distance
			     0.1, // alpha heading
			     0.1, // alpha speed
			     "/dev/stderr"); // progress stream
  if(0 > m_dynamicWindow_handle){
    cerr << "sfl_create_DynamicWindow(): " << m_dynamicWindow_handle << "\n";
    exit(EXIT_FAILURE);
  }
  
  m_odometry_handle = sfl_create_Odometry(m_hal_handle);
  if(0 > m_odometry_handle){
    cerr << "sfl_create_Odometry(): " << m_odometry_handle << "\n";
    exit(EXIT_FAILURE);
  }
  
  int scanner[] = { m_front_handle, m_rear_handle };
  m_multiscanner_handle =
    sfl_create_Multiscanner(m_odometry_handle, scanner, 2);
  if(0 > m_multiscanner_handle){
    cerr << "sfl_create_Multiscanner(): " << m_multiscanner_handle << "\n";
    exit(EXIT_FAILURE);
  }
  
  m_bubbleBand_handle =
    sfl_create_BubbleBand(m_robotModel_handle, m_odometry_handle,
			  m_multiscanner_handle, 4, 8, 4);
  if(0 > m_bubbleBand_handle){
    cerr << "sfl_create_BubbleBand(): " << m_bubbleBand_handle << "\n";
    exit(EXIT_FAILURE);
  }
  
  m_motionPlanner_handle =
    expo_create_MotionPlanner(m_motionController_handle,
			      m_dynamicWindow_handle,
			      m_multiscanner_handle,
			      m_robotModel_handle,
			      m_bubbleBand_handle,
			      m_odometry_handle);
  if(0 > m_motionPlanner_handle){
    cerr << "sfl_create_MotionPlanner(): " << m_motionPlanner_handle << "\n";
    exit(EXIT_FAILURE);
  }
}


Robot::
~Robot()
{
  expo_destroy_MotionPlanner(m_motionPlanner_handle);
  sfl_destroy_Multiscanner(m_multiscanner_handle);
  sfl_destroy_BubbleBand(m_bubbleBand_handle);
  sfl_destroy_Odometry(m_odometry_handle);
  sfl_destroy_DynamicWindow(m_dynamicWindow_handle);
  expo_destroy_MotionController(m_motionController_handle);
  sfl_destroy_RobotModel(m_robotModel_handle);
  sfl_destroy_Scanner(m_front_handle);
  sfl_destroy_Scanner(m_rear_handle);
  sfl_destroy_HAL(m_hal_handle);
}



void Robot::
SetGoal(const Goal & goal)
{
  const int res(expo_set_goal(m_motionPlanner_handle, 0.1, goal.X(), goal.Y(),
			      goal.Theta(), goal.Dr(), goal.Dtheta(),
			      goal.IsVia() ? 1 : 0));
  if(0 > res){
    cerr << "expo_set_goal(): " << res << "\n";
    exit(EXIT_FAILURE);
  }
}


void Robot::
PrepareAction(double timestep)
{
  const int res(expo_update_all(m_motionPlanner_handle, timestep));
  if(0 > res){
    cerr << "expo_update_all(): " << res << "\n";
    exit(EXIT_FAILURE);
  }  
}


void Robot::
DumpObstacles()
{
  shared_ptr<DynamicWindow> dwa(get_DynamicWindow(m_dynamicWindow_handle));
  if( ! dwa){
    cerr << "oops invalid get_DynamicWindow()\n";
    exit(EXIT_FAILURE);
  }
  dwa->DumpObstacles(cerr, "  ");
}


int hal_time_get(struct cwrap_hal_s * self, struct timespec * stamp)
{
  struct timeval tv;
  int res(gettimeofday(&tv, 0));
  if(res != 0)
    return -1;
  TIMEVAL_TO_TIMESPEC(&tv, stamp);
  return 0;
}


int hal_odometry_set(struct cwrap_hal_s * self,
		     double x, double y, double theta,
		     double sxx, double syy, double stt,
		     double sxy, double sxt, double syt)
{
  hal.x = x;
  hal.y = y;
  hal.theta = theta;
  hal.sxx = sxx;
  hal.syy = syy;
  hal.stt = stt;
  hal.sxy = sxy;
  hal.sxt = sxt;
  hal.syt = syt;
  return 0;
}


int hal_odometry_get(struct cwrap_hal_s * self,
		     struct timespec * stamp,
		     double * x, double * y, double * theta,
		     double * sxx, double * syy, double * stt,
		     double * sxy, double * sxt, double * syt)
{
  *x = hal.x;
  *y = hal.y;
  *theta = hal.theta;
  *sxx = hal.sxx;
  *syy = hal.syy;
  *stt = hal.stt;
  *sxy = hal.sxy;
  *sxt = hal.sxt;
  *syt = hal.syt;
  return hal_time_get(self, stamp);
}


int hal_speed_set(struct cwrap_hal_s * self, double qdl, double qdr)
{
  hal.qdl = qdl;
  hal.qdr = qdr;
  return 0;
}


int hal_speed_get(struct cwrap_hal_s * self, double * qdl, double * qdr)
{
  *qdl = hal.qdl;
  *qdr = hal.qdr;
  return 0;
}


int hal_scan_get(struct cwrap_hal_s * self,
		 int channel, double * rho, int rho_len,
		 struct timespec * t0, struct timespec * t1)
{
  if(0 > hal_time_get(self, t0))
    return -1;
  for(int ii(0); ii < rho_len; ++ii)
    rho[ii] = 0.5 + 0.01 * ii;
  return hal_time_get(self, t1);
}
