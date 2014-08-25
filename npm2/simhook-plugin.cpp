/* Nepumuk Mobile Robot Simulator v2
 * 
 * Copyright (C) 2014 Roland Philippsen. All rights resevred.
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

#include <npm2/Plugin.hpp>
#include <npm2/Object.hpp>
#include <npm2/Simulator.hpp>
#include <npm2/Factory.hpp>
#include <sfl/util/Line.hpp>
#include <limits>
#include <cmath>
#include <stdlib.h>
#include <sys/time.h>

using namespace std;


class Hook
  : public npm2::SimulatorHook,
    public fpplib::Configurable
{
public:
  Hook ();
  
  virtual void preActuation (ostream & err);
  virtual void preSensing (ostream & err);
  virtual void preProcessing (ostream & err);
  
  npm2::Object * world_;
  npm2::Object * container_;
  sfl::Line bounds_;
  bool container_attached_;
};

static Hook hook;


//////////////////////////////////////////////////


int npm2_plugin_init ()
{
  npm2::Factory::instance().declareSingleton <Hook> ("ContainerTeleport", &hook);
  npm2::Simulator::instance()->addHook (false, &hook);
  
  struct timeval tt;
  gettimeofday (&tt, NULL);
  srand (tt.tv_usec);
  
  return 0;
}


//////////////////////////////////////////////////


Hook::
Hook ()
  : fpplib::Configurable ("simhook"),
    world_ (0),
    container_ (0),
    bounds_ (0.0, 0.0, 0.0, 0.0),
    container_attached_ (false)
{
  reflectSlot ("world", &world_);
  reflectSlot ("container", &container_);
  reflectParameter ("bounds", &bounds_);
}


void Hook::
preActuation (ostream & err)
{
  if ( ! world_) {
    err << "ContainerTeleport: undefined world\n";
    return;
  }
  if ( ! container_) {
    err << "ContainerTeleport: undefined container\n";
    return;
  }
  if (bounds_.X1() - bounds_.X0() < 1e-3) {
    err << "ContainerTeleport: undefined or invalid bounds\n";
    return;
  }
  
  if (container_attached_) {
    if (container_->getParent() == world_) {
      container_attached_ = false;
      static double const nn (1.0 / std::numeric_limits <unsigned int> ::max());
      double const px (bounds_.X0() + (bounds_.X1() - bounds_.X0()) * rand() * nn);
      double const py (bounds_.Y0() + (bounds_.Y1() - bounds_.Y0()) * rand() * nn);
      double const pth (2 * M_PI * rand() * nn);
      container_->mount_.Set (px, py, pth);
      container_->motion_.Set (0.0, 0.0, 0.0);
    }
  }
  else {
    if (container_->getParent() != world_) {
      container_attached_ = true;
    }
  }

}


void Hook::
preSensing (ostream & err)
{
}


void Hook::
preProcessing (ostream & err)
{
}
