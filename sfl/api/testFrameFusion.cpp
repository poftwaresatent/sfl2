#include <sfl/api/FrameFusion.hpp>
#include <sfl/util/numeric.hpp>
#include <vector>
#include <iostream>
#include <err.h>

using namespace sfl;
using namespace std;

static vector<Frame> true_pose;
static vector<Frame> odom_noise;
static vector<Frame> raw_odom;

static void init();

int main(int argc, char ** argv)
{
  init();
  Timestamp const timestep(0, 3000000);
  FrameFusion ff(100, 10, 100, 100, &cout, &cout);
  Timestamp now;
  for (size_t ii(0); ii < odom_noise.size(); ++ii) {
    ff.AddRawOdometry(now, raw_odom[ii]);
    if ( ! ff.UpdateOdomCorrection(now, true_pose[ii]))
      errx(EXIT_FAILURE, "zonk ff.UpdateOdomCorrection!");
    Frame const ext(ff.Extrapolate(now));
    Frame delta(true_pose[ii]);
    ext.From(delta);
    double const diff(absval(delta.Theta()) + absval(delta.X()) + absval(delta.Y()));
    if (diff < 0.001)
      cout << "OK  ";
    else
      cout << "grr ";
    printf("t %5.2f %5.2f %5.2f  x %5.2f %5.2f %5.2f  d %5.2f %5.2f %5.2f  n %5.2f %5.2f %5.2f\n",
	   true_pose[ii].X(), true_pose[ii].Y(), true_pose[ii].Theta(),
	   ext.X(), ext.Y(), ext.Theta(),
	   delta.X(), delta.Y(), delta.Theta(),
	   odom_noise[ii].X(), odom_noise[ii].Y(), odom_noise[ii].Theta());
    now += timestep;
  }
}

void init()
{
  double const radius(10);
  for (double phi(0); phi < M_PI; phi += 0.01 * M_PI) {
    true_pose.push_back(Frame(radius * cos(phi),
			      radius * sin(phi),
			      phi));
    odom_noise.push_back(Frame(phi * 0.2,
			       phi * 0.07 - 0.1,
			       0.3 - phi * 0.1));
  }
  for (size_t ii(0); ii < odom_noise.size(); ++ii) {
    Frame raw(true_pose[ii]);
    odom_noise[ii].To(raw);
    raw_odom.push_back(raw);
  }
}
