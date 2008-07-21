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

#ifdef NPM_HAVE_GENOM
# include <npm/laas/LAAS.hpp>
#endif // NPM_HAVE_GENOM

#ifdef NPM_HAVE_XCF
# include <npm/biron/Biron.hpp>
#endif // NPM_HAVE_XCF

#ifdef NPM_HAVE_ESTAR
# include <npm/estar/Esbot.hpp>
#endif // NPM_HAVE_ESTAR

#ifdef NPM_HAVE_ASL
# include <npm/asl/Smart.hpp>
# include <npm/asl/Borox.hpp>
#endif // NPM_HAVE_ASL

#ifdef NPM_HAVE_ROS
# include <npm/ros/ROSbot.hpp>
#endif // NPM_HAVE_ROS

//#include <npm/theater/TheaterRobot.hpp>
////#include <npm/braitenberg/Braitenberg.hpp>


using namespace boost;


namespace npm {


  shared_ptr<RobotClient> RobotFactory::
  Create(shared_ptr<RobotDescriptor> descriptor, const World & world)
  {
    RobotClient * rob(0);

    if(descriptor->model == "robox")
      rob = Robox::Create(descriptor, world);

    else if(descriptor->model == "visitor")
      rob = new Visitor(descriptor, world);

#ifdef NPM_HAVE_XCF
    else if(descriptor->model == "biron")
      rob = new Biron(descriptor, world);
#endif // NPM_HAVE_XCF

#ifdef NPM_HAVE_ESTAR
    else if(descriptor->model == "esbot")
      rob = new Esbot(descriptor, world);
#endif // NPM_HAVE_ESTAR

#ifdef NPM_HAVE_ASL
    else if (descriptor->model == "smart")
      rob = new Smart(descriptor, world);
    else if (descriptor->model == "borox")
      rob = new Borox(descriptor, world);
#endif // NPM_HAVE_ASL

#ifdef NPM_HAVE_GENOM
    else if(descriptor->model == "jido")
      rob = new Jido(descriptor, world);
    else if(descriptor->model == "rackham")
      rob = new Rackham(descriptor, world);
#endif // NPM_HAVE_GENOM

#ifdef NPM_HAVE_ROS
    else if ((descriptor->model == "ros")
	     || (descriptor->model == "ROS")
	     || (descriptor->model == "ROSbot"))
      rob = new ROSbot(descriptor, world);
#endif // NPM_HAVE_ROS

    return shared_ptr<RobotClient>(rob);
  }

}
