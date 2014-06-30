#include <npm2/DifferentialDrive.hpp>
#include <npm2/RayDistanceSensor.hpp>
#include <npm2/gfx.hpp>
#include <iostream>

#include <cmath>
#include <stdio.h>


using namespace npm2;


static Object world ("world");
static Object base ("base");
static DifferentialDrive drive;
static RayDistanceSensor sensor ("sensor");
static double const timestep (0.1);

static double mx0, my0, mx1, my1;

static enum {
  PAUSE,
  STEP,
  RUN
} state;


static void recurse_draw (Object const * obj)
{
  gfx::set_pen (1.0, 0.0, 0.0, 0.0, 1.0);
  Body::lines_t const & lines (obj->body_.getLines());
  for (size_t il(0); il < lines.size(); ++il) {
    gfx::draw_line (lines[il].X0(), lines[il].Y0(), lines[il].X1(), lines[il].Y1());
  }
  for (Object::child_iterator_t ic(obj->childBegin()); ic != obj->childEnd(); ++ic) {
    recurse_draw (*ic);
  }
}


static void cb_draw ()
{
  BBox const & bbox (world.getBBox());
  if (bbox.isValid()) {
    static double const margin (0.1);
    gfx::set_view (bbox.x0() - margin, bbox.y0() - margin, bbox.x1() + margin, bbox.y1() + margin);
    recurse_draw (&world);
  }
}


static void cb_mouse (double mx, double my, int flags)
{
  if (flags & gfx::MOUSE_PRESS) {
    mx0 = mx;
    my0 = my;
  }
  else if (flags & gfx::MOUSE_DRAG) {
    mx1 = mx;
    my1 = my;
  }
}


static void tick ()
{
  static size_t count (0);
  
  drive.integrate (timestep);
  
  world.updateTransform ();
  sensor.sensorReset ();
  world.updateSensor (&sensor);
  
  printf ("% 3zu    %+6.3f  %+6.3f  %+6.3f    %+6.3f\n",
	  count++,
	  base.getGlobal().X(), base.getGlobal().Y(), base.getGlobal().Theta(),
	  sensor.distance_);
  
  drive.setSpeed (0.1, 0.2);
}


static void cb_idle ()
{
  switch (state) {
  case PAUSE:
    break;
  case STEP:
    tick();
    state = PAUSE;
    break;
  case RUN:
  default:
    tick();
  }
}


static void cb_pause ()
{
  if (state == RUN) {
    state = PAUSE;
  }
  else {
    state = RUN;
  }
}


static void cb_next ()
{
  state = STEP;
}


int main (int argc, char ** argv)
{
  //////////////////////////////////////////////////
  
  world.body_.addLine (-5.0, -5.0,  5.0, -5.0);
  world.body_.addLine ( 5.0, -5.0,  5.0,  5.0);
  world.body_.addLine ( 5.0,  5.0, -5.0,  5.0);
  world.body_.addLine (-5.0,  5.0, -5.0, -5.0);
  
  base.body_.addLine (-0.2, -0.4,  0.4, -0.2);
  base.body_.addLine (-0.2,  0.4,  0.4,  0.2);
  base.body_.addLine ( 0.4, -0.2,  0.4,  0.2);
  base.body_.addLine (-0.2,  0.4, -0.2, -0.4);
  base.setParent (&world);
  base.mount_.Set (0.0, -2.5, 0.0);
  
  drive.object_ = &base;
  
  sensor.max_distance_ = 10000000.0;
  sensor.setParent (&base);
  sensor.mount_.Set (0.2, 0.0, 0.0);
  
  // Make sure we can draw something before we start running.
  //
  world.updateTransform ();
  
  //////////////////////////////////////////////////
  
  mx0 =  0.0;
  my0 =  0.0;
  mx1 =  0.5;
  my1 = -0.5;
  
  // This just enables (rather verbose) debug messages from the gfx
  // wrapper.  It is optional but can be rather useful.
  gfx::debug (&cout);
  
  // Adding a custom button needs to happen before gfx::main is called.
  gfx::add_button ("run/pause", cb_pause);
  
  // Adding a custom button needs to happen before gfx::main is called.
  gfx::add_button ("next", cb_next);
  
  // The gfx::main function enters the GUI processing loop.  It
  // returns when the user has clicked the "quit" button (which gets
  // added automatically) or otherwise exited the GUI application.
  gfx::main (argv[0], cb_idle, cb_draw, cb_mouse);
}
