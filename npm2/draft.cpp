/* Nepumuk Mobile Robot Simulator v2
 *
 * Copyright (C) 2014 Roland Philippsen. All rights reserved.
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

#include <npm2/DifferentialDrive.hpp>
#include <npm2/DifferentialTrailerDrive.hpp>
#include <npm2/RevoluteServo.hpp>
#include <npm2/RayDistanceSensor.hpp>
#include <npm2/gfx.hpp>
#include <sfl/util/numeric.hpp>
#include <iostream>

#include <cmath>
#include <stdio.h>


using namespace npm2;


static Object world ("world");

static Object alice ("alice");
static DifferentialDrive alice_drive;
static RevoluteServo alice_servo;
static RayDistanceSensor alice_sensor ("alice_sensor");

static Object bob_tractor ("bob_tractor");
static Object bob_trailer ("bob_trailer");
static DifferentialTrailerDrive bob_drive;

static double const timestep (0.1);

static double mx0, my0, mx1, my1;

static enum {
  PAUSE,
  STEP,
  RUN
} state (STEP);


static void recurse_draw (Object const * obj)
{
  gfx::set_pen (2.0, 0.0, 0.0, 0.0, 1.0);
  Body::lines_t const & lines (obj->body_.getLines());
  for (size_t il(0); il < lines.size(); ++il) {
    gfx::draw_line (lines[il].X0(), lines[il].Y0(), lines[il].X1(), lines[il].Y1());
  }
  
  RayDistanceSensor const * rds (dynamic_cast <RayDistanceSensor const *> (obj));
  if (rds) {
    gfx::set_pen (1.0, 1.0, 0.0, 0.0, 1.0);
    gfx::draw_line (rds->getGlobal().X(),
		    rds->getGlobal().Y(),
		    rds->getGlobal().X() + rds->distance_ * rds->getGlobal().Costheta(),
		    rds->getGlobal().Y() + rds->distance_ * rds->getGlobal().Sintheta());
  }
  
  for (Object::child_iterator_t ic(obj->childBegin()); ic != obj->childEnd(); ++ic) {
    recurse_draw (*ic);
  }
}


static void cb_draw ()
{
  BBox const & bbox (world.getBBox());
  if ( ! bbox.isValid()) {
    return;
  }
  
  // world
  
  static double const margin (0.1);
  gfx::set_view (bbox.x0() - margin, bbox.y0() - margin, bbox.x1() + margin, bbox.y1() + margin);
  recurse_draw (&world);
  
  // bob drive
  
  gfx::set_pen (2.0, 0.0, 0.5, 0.5, 1.0);
  double x0, y0, x1, y1;
  x0 =  0.0;
  y0 = -bob_drive.wheel_base_ / 2.0;
  x1 =  0.0;
  y1 = -y0;
  bob_drive.tractor_->getGlobal().To (x0, y0);
  bob_drive.tractor_->getGlobal().To (x1, y1);
  gfx::draw_line (x0, y0, x1, y1);
  x0 = -bob_drive.wheel_radius_;
  y0 =  bob_drive.wheel_base_ / 2.0;
  x1 = -x0;
  y1 =  y0;
  bob_drive.tractor_->getGlobal().To (x0, y0);
  bob_drive.tractor_->getGlobal().To (x1, y1);
  gfx::draw_line (x0, y0, x1, y1);
  x0 = -bob_drive.wheel_radius_;
  y0 = -bob_drive.wheel_base_ / 2.0;
  x1 = -x0;
  y1 =  y0;
  bob_drive.tractor_->getGlobal().To (x0, y0);
  bob_drive.tractor_->getGlobal().To (x1, y1);
  gfx::draw_line (x0, y0, x1, y1);
  x0 = -bob_drive.hitch_offset_;
  y0 =  0.0;
  x1 =  0.0;
  y1 =  0.0;
  bob_drive.tractor_->getGlobal().To (x0, y0);
  bob_drive.tractor_->getGlobal().To (x1, y1);
  gfx::draw_line (x0, y0, x1, y1);
  x0 = -bob_drive.hitch_offset_;
  y0 =  0.0;
  x1 =  x0 - bob_drive.trailer_arm_ * cos (bob_drive.getTrailerAngle());
  y1 =     - bob_drive.trailer_arm_ * sin (bob_drive.getTrailerAngle());
  bob_drive.tractor_->getGlobal().To (x0, y0);
  bob_drive.tractor_->getGlobal().To (x1, y1);
  gfx::draw_line (x0, y0, x1, y1);

  gfx::set_pen (2.0, 0.0, 0.6, 0.4, 1.0);
  x0 =  0.0;
  y0 = -bob_drive.wheel_base_ / 2.0;
  x1 =  0.0;
  y1 = -y0;
  bob_drive.trailer_->getGlobal().To (x0, y0);
  bob_drive.trailer_->getGlobal().To (x1, y1);
  gfx::draw_line (x0, y0, x1, y1);
  x0 = -bob_drive.wheel_radius_;
  y0 =  bob_drive.wheel_base_ / 2.0;
  x1 = -x0;
  y1 =  y0;
  bob_drive.trailer_->getGlobal().To (x0, y0);
  bob_drive.trailer_->getGlobal().To (x1, y1);
  gfx::draw_line (x0, y0, x1, y1);
  x0 = -bob_drive.wheel_radius_;
  y0 = -bob_drive.wheel_base_ / 2.0;
  x1 = -x0;
  y1 =  y0;
  bob_drive.trailer_->getGlobal().To (x0, y0);
  bob_drive.trailer_->getGlobal().To (x1, y1);
  gfx::draw_line (x0, y0, x1, y1);
  
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
  
  alice_drive.integrate (timestep);
  alice_servo.integrate (timestep);
  bob_drive.integrate (timestep);
  
  world.updateTransform ();
  alice_sensor.sensorReset ();
  world.updateSensor (&alice_sensor);
  
  // printf ("% 3zu    %+6.3f  %+6.3f  %+6.3f    %+6.3f\n",
  // 	  count++,
  // 	  alice.getGlobal().X(), alice.getGlobal().Y(), alice.getGlobal().Theta(),
  // 	  alice_sensor.distance_);
  
  alice_drive.setSpeed (0.02, 0.04);
  
  double thref;
  if (bob_tractor.getGlobal().X() > bob_tractor.getGlobal().Y()) {
    if (bob_tractor.getGlobal().X() > - bob_tractor.getGlobal().Y()) {
      thref = 105.0 * M_PI / 180.0;
      //      printf ("AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA\n");
    }
    else {
      thref = 15.0 * M_PI / 180.0;
      //      printf ("BBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBB\n");
    }
  }
  else {
    if (bob_tractor.getGlobal().X() > - bob_tractor.getGlobal().Y()) {
      thref = 195.0 * M_PI / 180.0;
      //      printf ("CCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCC\n");
    }
    else {
      thref = -75.0 * M_PI / 180.0;
      //      printf ("DDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDD\n");
    }
  }
  double const dhead (mod2pi (thref - bob_tractor.getGlobal().Theta()));
  //printf ("%g\n", dhead);
  static double const dth (5.0 * M_PI / 180.0);
  if (fabs (dhead) <= dth) {
    bob_drive.setSpeed (0.02, 0.02);
  }
  else if (dhead > 0.0) {
    bob_drive.setSpeed (0.0, 0.02);
  }
  else {
    bob_drive.setSpeed (0.02, 0.0);
  }
  
  static double amp (5.0 * M_PI / 180.0);
  static double omg (2.0 * M_PI / 5.0);
  alice_servo.setAngle (amp * cos (omg * count * timestep));
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
  
  //////////////////////////////////////////////////
  
  alice.body_.addLine (-0.2, -0.4,  0.4, -0.2);
  alice.body_.addLine (-0.2,  0.4,  0.4,  0.2);
  alice.body_.addLine ( 0.4, -0.2,  0.4,  0.2);
  alice.body_.addLine (-0.2,  0.4, -0.2, -0.4);
  alice.setParent (&world);
  alice.mount_.Set (0.0, -2.5, 0.0);
  
  alice_drive.object_ = &alice;
  
  alice_servo.object_ = &alice_sensor;
  
  alice_sensor.max_distance_ = 10000000.0;
  alice_sensor.setParent (&alice);
  alice_sensor.mount_.Set (0.2, 0.0, 0.0);
  
  //////////////////////////////////////////////////
  
  bob_drive.tractor_ = &bob_tractor;
  bob_drive.trailer_ = &bob_trailer;
  
  static double const hitch_offset (0.3);
  static double const trailer_arm (1.0);
  
  bob_drive.wheel_radius_ = 0.2;
  bob_drive.wheel_base_ = 0.4;
  bob_drive.hitch_offset_ = hitch_offset;
  bob_drive.trailer_arm_ = trailer_arm;
  
  bob_tractor.body_.addLine (-hitch_offset, -0.3,           0.4, -0.3);
  bob_tractor.body_.addLine (-hitch_offset,  0.3,           0.4,  0.3);
  bob_tractor.body_.addLine (-hitch_offset, -0.3, -hitch_offset,  0.3);
  bob_tractor.body_.addLine (          0.4, -0.3,           0.4,  0.3);
  bob_tractor.setParent (&world);
  
  bob_tractor.mount_.Set (0.0, 2.5, M_PI);
  
  bob_trailer.body_.addLine (       -0.3, -0.3, trailer_arm, -0.3);
  bob_trailer.body_.addLine (       -0.3,  0.3, trailer_arm,  0.3);
  bob_trailer.body_.addLine (       -0.3, -0.3,        -0.3,  0.3);
  bob_trailer.body_.addLine (trailer_arm, -0.3, trailer_arm,  0.3);
  bob_trailer.setParent (&bob_tractor);
  
  // Make sure we can draw something before we start running.
  //
  world.updateTransform ();
  
  //////////////////////////////////////////////////
  
  mx0 =  0.0;
  my0 =  0.0;
  mx1 =  0.5;
  my1 = -0.5;
  
  // // This just enables (rather verbose) debug messages from the gfx
  // // wrapper.  It is optional but can be rather useful.
  // gfx::debug (&cout);
  
  // Adding a custom button needs to happen before gfx::main is called.
  gfx::add_button ("run/pause", cb_pause);
  
  // Adding a custom button needs to happen before gfx::main is called.
  gfx::add_button ("next", cb_next);
  
  // The gfx::main function enters the GUI processing loop.  It
  // returns when the user has clicked the "quit" button (which gets
  // added automatically) or otherwise exited the GUI application.
  gfx::main (argv[0], cb_idle, cb_draw, cb_mouse);
}
