#include <sfl/util/Frame.hpp>
#include <sfl/util/Line.hpp>
#include <sfl/util/numeric.hpp>
#include <set>
#include <vector>

#include <stdio.h>


namespace npm2 {
  
  using namespace sfl;
  using namespace std;
  
  
  class Body
  {
  public:
    void transformTo (Frame const & global)
    {
      if (global_lines_.size() != local_lines_.size()) {
	global_lines_.resize (local_lines_.size());
      }
      for (size_t il(0); il < local_lines_.size(); ++il) {
	global_lines_[il] = local_lines_[il];
	global_lines_[il].TransformTo (global);
      }
    }
    
    typedef vector <Line> lines_t;
    lines_t local_lines_;
    lines_t global_lines_;
  };
  

  class Sensor;
  
  class Object
  {
  public:
    Object ()
      : parent_ (0)
    {
    }

    virtual ~Object() {}
    
    void setParent (Object * obj)
    {
      if (parent_) {
	parent_->children_.erase (this);
      }
      parent_ = obj;
      if (parent_) {
	parent_->children_.insert (this);
      }
    }
    
    /* assumes parent has been updated, and recurses into all
       children */
    void updateTransform ()
    {
      global_ = motion_;
      mount_.To (global_);
      if (parent_) {
	parent_->global_.To (global_);
      }
      body_.transformTo (global_);
      
      for (children_t::iterator ic (children_.begin()); ic != children_.end(); ++ic) {
	(*ic)->updateTransform();
      }
    }

    void updateSensor (Sensor * sensor);
    
    typedef set <Object*> children_t;
    
    Body body_;
    Frame mount_;
    Frame motion_;
    Frame global_;
    Object * parent_;
    children_t children_;
  };
  
  
  class Actuator
  {
  public:
    virtual ~Actuator () {}
    
    virtual void integrate (double dt) = 0;
  };
  
  
  class DifferentialDrive
    : public Actuator
  {
  public:
    DifferentialDrive ()
      : radius_left_ (1.0),
	radius_right_ (1.0),
	wheel_base_ (1.0),
	speed_left_ (0.0),
	speed_right_ (0.0),
	object_ (0)
    {
    }
    
    void setSpeed (double wl, double wr)
    {
      speed_left_ = wl;
      speed_right_ = wr;
    }
    
    virtual void integrate (double dt)
    {
      if ( ! object_) {
	return;
      }
      
      double const dl (radius_left_ * speed_left_);
      double const dr (radius_right_ * speed_right_);
      double const vtrans ((dl + dr) / 2.0);
      double const vrot ((dr - dl) / wheel_base_);
      double dx (vtrans * dt);
      double dy (0.0);
      double dth (vrot * dt);
      
      object_->motion_.RotateTo (dx, dy);
      object_->motion_.Add (dx, dy, dth);
    }
    
    double radius_left_;
    double radius_right_;
    double wheel_base_;
    double speed_left_;
    double speed_right_;
    Object * object_;
  };
  
  
  class Sensor
    : public Object
  {
  public:
    virtual void sensorReset () = 0;
    virtual void sensorUpdate (Body const & body) = 0;
  };
  
  
  void Object::
  updateSensor (Sensor * sensor)
  {
    sensor->sensorUpdate (body_);
    for (children_t::iterator ic (children_.begin()); ic != children_.end(); ++ic) {
      (*ic)->updateSensor (sensor);
    }
  }
  
  
  class RayDistanceSensor
    : public Sensor
  {
  public:
    RayDistanceSensor ()
      : distance_ (1.0),
	max_distance_ (1.0)
    {
    }
    
    virtual void sensorReset ()
    {
      distance_ = max_distance_;
    }
    
    virtual void sensorUpdate (Body const & body)
    {
      // deciding which bodies to ignore could be smarter...
      //
      if ((&body == &body_) || (parent_ && (&body == &parent_->body_))) {
	return;
      }
      
      for (size_t il(0); il < body.global_lines_.size(); ++il) {
	Line const & ll (body.global_lines_[il]);
	double const dd (LineRayIntersect (ll.X0(), ll.Y0(), ll.X1(), ll.Y1(),
					   global_.X(), global_.Y(),
					   global_.Costheta(), global_.Sintheta()));
	if ((dd > 0) && (dd < distance_)) {
	  distance_ = dd;
	}
      }
    }
    
    double distance_;
    double max_distance_;
  };
  
}


using namespace npm2;

int main (int argc, char ** argv)
{
  Object world;
  world.body_.local_lines_.push_back (Line (-5.0, -5.0,  5.0, -5.0));
  world.body_.local_lines_.push_back (Line ( 5.0, -5.0,  5.0,  5.0));
  world.body_.local_lines_.push_back (Line ( 5.0,  5.0, -5.0,  5.0));
  world.body_.local_lines_.push_back (Line (-5.0,  5.0, -5.0, -5.0));
  
  Object base;
  base.body_.local_lines_.push_back (Line (-0.2, -0.4,  0.4, -0.2));
  base.body_.local_lines_.push_back (Line (-0.2,  0.4,  0.4,  0.2));
  base.body_.local_lines_.push_back (Line ( 0.4, -0.2,  0.4,  0.2));
  base.body_.local_lines_.push_back (Line (-0.2,  0.4, -0.2, -0.4));
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
	    ii, base.global_.X(), base.global_.Y(), base.global_.Theta(), sensor.distance_);
    
    drive.setSpeed (0.1, 0.2);
    drive.integrate (dt);
  }
}
