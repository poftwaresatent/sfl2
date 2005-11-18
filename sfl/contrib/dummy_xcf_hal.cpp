#include "xcf_hal.h"
#include <sfl/util/numeric.hpp>
#include <iostream>


using sfl::epsilon;
using sfl::absval;
using std::cout;


int xcf_odometry_init()
{
  cout << "HELLO from xcf_odometry_init()!\n";
  return 0;
}


int xcf_odometry_get(double * x, double * y, double * theta_rad,
		     uint64_t * timestamp_ms)
{
  //  cout << "HELLO from xcf_odometry_get()!\n";
  *x = 0;
  *y = 0;
  *theta_rad = 0;
  *timestamp_ms = 0;
  return 0;
}


int xcf_odometry_end()
{
  cout << "HELLO from xcf_odometry_end()!\n";
  return 0;
}


int xcf_speed_init()
{
  cout << "HELLO from xcf_speed_init()!\n";
  return 0;
}


int xcf_speed_set(double v, double w)
{
  static double oldv(0), oldw(0);
  if((absval(v - oldv) > epsilon)
     || (absval(w - oldw) > epsilon))
    cout << "INFO from xcf_speed_set(): " << v << "\t" << w << "\n";
  oldv = v;
  oldw = w;
  return 0;
}


int xcf_speed_end()
{
  cout << "HELLO from xcf_speed_end()!\n";
  return 0;
}


int xcf_scan_init(int channel)
{
  cout << "HELLO from xcf_scan_init()!\n";
  return 0;
}


int xcf_scan_get(int channel, double * rho, int rho_len,
		 uint64_t * timestamp_ms)
{
  //  cout << "HELLO from xcf_scan_get()!\n";
  for(int i(0); i < rho_len; ++i)
    rho[i] = 0.5;
  *timestamp_ms = 0;
  return 0;
}


int xcf_scan_end(int channel)
{
  cout << "HELLO from xcf_scan_end()!\n";
  return 0;
}


void xcf_hal_dryrun_on(FILE * stream)
{
  /* dummy */
}


void xcf_hal_dryrun_off()

{
  /* dummy */
}

