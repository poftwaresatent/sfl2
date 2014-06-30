#include <npm2/DifferentialDrive.hpp>
#include <npm2/RayDistanceSensor.hpp>
#include <npm2/gfx.hpp>
#include <iostream>

#include <cmath>
#include <stdio.h>


using namespace npm2;


static Object world;
static Object base;
static DifferentialDrive drive;
static RayDistanceSensor sensor;
static double const timestep (0.1);

static double mx0, my0, mx1, my1;


static void cb_draw ()
{
  gfx::set_pen (1.0, 0.0, 0.0, 0.0, 1.0); // width, red, green, blue, alpha
  gfx::draw_arc (1.0, 0.0, 1.0, 0.0, 2 * M_PI);
  
  gfx::set_pen (1.0, 0.5, 0.5, 0.5, 1.0);
  gfx::fill_arc (-1.0, 0.0, 1.0, 0.0, 2 * M_PI);
  
  gfx::set_pen (1.0, 1.0, 0.0, 0.0, 1.0);
  gfx::draw_line (0.0, 0.0, 1.0, 0.0);
  
  gfx::set_pen (1.0, 0.0, 1.0, 0.0, 1.0);
  gfx::draw_line (0.0, 0.0, 0.0, 1.0);
  
  gfx::set_pen (1.0, 0.0, 0.0, 1.0, 1.0);
  gfx::draw_line (0.0, 0.0, 1.0, 1.0);
  gfx::draw_line (1.0, 0.0, 0.0, 1.0);
  
  gfx::set_pen (3.0, 0.5, 0.0, 0.0, 1.0);
  gfx::draw_line (mx0, my0, mx1, my1);
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


static void cb_next ()
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
  
  //////////////////////////////////////////////////
  
  mx0 =  0.0;
  my0 =  0.0;
  mx1 =  0.5;
  my1 = -0.5;
  
  // This just enables (rather verbose) debug messages from the gfx
  // wrapper.  It is optional but can be rather useful.
  gfx::debug (&cout);
  
  // Adding a custom button needs to happen before gfx::main is called.
  gfx::add_button ("next", cb_next);
  
  // The gfx::main function enters the GUI processing loop.  It
  // returns when the user has clicked the "quit" button (which gets
  // added automatically) or otherwise exited the GUI application.
  gfx::main (argv[0], cb_draw, cb_mouse);
}
