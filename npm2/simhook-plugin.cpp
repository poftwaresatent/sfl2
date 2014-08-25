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
};

static Hook hook;


//////////////////////////////////////////////////


int npm2_plugin_init ()
{
  npm2::Factory::instance().declareSingleton <Hook> ("ContainerTeleport", &hook);
  npm2::Simulator::instance()->addHook (false, &hook);
  return 0;
}


//////////////////////////////////////////////////


Hook::
Hook ()
  : fpplib::Configurable ("simhook")
{
  reflectSlot ("world", &world_);
  reflectSlot ("container", &container_);
  reflectParameter ("bounds", &bounds_);
}


void Hook::
preActuation (ostream & err)
{
  err << "Hello from the pre actuation hook\n";
}


void Hook::
preSensing (ostream & err)
{
  err << "Hello from the pre sensing hook\n";
}


void Hook::
preProcessing (ostream & err)
{
  err << "Hello from the pre processing hook\n";
}
