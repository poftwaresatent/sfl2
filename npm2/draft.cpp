#include <npm2/DifferentialDrive.hpp>
#include <npm2/RayDistanceSensor.hpp>

#include <stdio.h>


using namespace npm2;

int main (int argc, char ** argv)
{
  Object world;
  world.body_.addLine (-5.0, -5.0,  5.0, -5.0);
  world.body_.addLine ( 5.0, -5.0,  5.0,  5.0);
  world.body_.addLine ( 5.0,  5.0, -5.0,  5.0);
  world.body_.addLine (-5.0,  5.0, -5.0, -5.0);
  
  Object base;
  base.body_.addLine (-0.2, -0.4,  0.4, -0.2);
  base.body_.addLine (-0.2,  0.4,  0.4,  0.2);
  base.body_.addLine ( 0.4, -0.2,  0.4,  0.2);
  base.body_.addLine (-0.2,  0.4, -0.2, -0.4);
  base.setParent (&world);
  base.mount_.Set (0.0, -2.5, 0.0);
  
  DifferentialDrive drive;
  drive.object_ = &base;
  
  RayDistanceSensor sensor;
  sensor.max_distance_ = 10000000.0;
  sensor.setParent (&base);
  sensor.mount_.Set (0.2, 0.0, 0.0);
  
  double const dt (0.1);
  for (size_t ii(0); ii < 500; ++ii) {
    world.updateTransform ();
    sensor.sensorReset ();
    world.updateSensor (&sensor);

    printf ("% 3zu    %+6.3f  %+6.3f  %+6.3f    %+6.3f\n",
	    ii,
	    base.getGlobal().X(), base.getGlobal().Y(), base.getGlobal().Theta(),
	    sensor.distance_);
    
    drive.setSpeed (0.1, 0.2);
    drive.integrate (dt);
  }
}
