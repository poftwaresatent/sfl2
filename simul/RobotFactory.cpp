/* 
 * Copyright (C) 2004
 * Swiss Federal Institute of Technology, Lausanne. All rights reserved.
 * 
 * Author: Roland Philippsen <roland dot philippsen at gmx dot net>
 *         Autonomous Systems Lab <http://asl.epfl.ch/>
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


#include "RobotFactory.hpp"
#include <npm/common/RobotDescriptor.hpp>
#include <npm/visitor/Visitor.hpp>
#include <npm/robox/Robox.hpp>
#include <npm/laas/LAAS.hpp>

#ifdef BUILD_BIRON
# include <npm/biron/Biron.hpp>
#endif // BUILD_BIRON

#ifdef BUILD_ESTAR
# include <npm/estar/Esbot.hpp>
#endif // BUILD_ESTAR

//#include <npm/theater/TheaterRobot.hpp>
////#include <npm/braitenberg/Braitenberg.hpp>


using namespace npm;
using namespace boost;


shared_ptr<RobotClient> RobotFactory::
Create(shared_ptr<RobotDescriptor> descriptor, const World & world)
{
  RobotClient * rob(0);
  if(descriptor->model == "robox")
    rob = Robox::Create(descriptor, world);
  else if(descriptor->model == "visitor")
    rob = new Visitor(descriptor, world);
#ifdef BUILD_BIRON
  else if(descriptor->model == "biron")
    rob = new Biron(descriptor, world);
#endif // BUILD_BIRON
#ifdef BUILD_ESTAR
  else if(descriptor->model == "esbot")
    rob = new Esbot(descriptor, world);
#endif // BUILD_ESTAR
  else if(descriptor->model == "jido")
    rob = new Jido(descriptor, world);
  else if(descriptor->model == "rackham")
    rob = new Rackham(descriptor, world);
  return shared_ptr<RobotClient>(rob);
}
