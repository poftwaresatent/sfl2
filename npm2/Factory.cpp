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

#include "Factory.hpp"
#include "Simulator.hpp"
#include <npm2/Object.hpp>
#include <npm2/DifferentialDrive.hpp>
#include <npm2/DifferentialTrailerDrive.hpp>
#include <npm2/RayDistanceSensor.hpp>
#include <npm2/RevoluteServo.hpp>
#include <npm2/AliceProcess.hpp>
#include <npm2/BobProcess.hpp>
#include <npm2/CharlieProcess.hpp>
#include <npm2/View.hpp>
#include <npm2/ObjectCamera.hpp>
#include <npm2/ObjectDrawing.hpp>
#include <npm2/GenericDrive.hpp>
#include <npm2/KinematicControl.hpp>
#include <sfl/util/Line.hpp>
#include <sfl/util/Frame.hpp>
#include <sfl/api/Goal.hpp>
#include <fpplib/yaml_parser.hpp>
#include <err.h>


namespace sfl {
  
  void operator >> (const YAML::Node & node, Line & ll)
  {
    node[0] >> ll.p0._x;
    node[1] >> ll.p0._y;
    node[2] >> ll.p1._x;
    node[3] >> ll.p1._y;
  }
  
  void operator >> (const YAML::Node & node, Frame & ff)
  {
    double xx, yy, th;
    node[0] >> xx;
    node[1] >> yy;
    node[2] >> th;
    ff.Set (xx, yy, th);
  }
  
  void operator >> (const YAML::Node & node, Goal & gg)
  {
    double xx, yy, th, dr, dth;
    node[0] >> xx;
    node[1] >> yy;
    node[2] >> th;
    node[3] >> dr;
    node[4] >> dth;
    gg.Set (xx, yy, th, dr, dth);
  }
  
}

namespace npm2 {
  
  
  Factory::
  Factory()
    : parser_ (*this)
  {
    declare <Object> ("Object");
    declare <Simulator> ("Simulator");
    declare <DifferentialDrive> ("DifferentialDrive");
    declare <DifferentialTrailerDrive> ("DifferentialTrailerDrive");
    declare <RayDistanceSensor> ("RayDistanceSensor");
    declare <RevoluteServo> ("RevoluteServo");
    declare <AliceProcess> ("AliceProcess");
    declare <BobProcess> ("BobProcess");
    declare <CharlieProcess> ("CharlieProcess");
    declare <View> ("View");
    declare <ObjectCamera> ("ObjectCamera");
    declare <ObjectDrawing> ("ObjectDrawing");
    declare <GenericDrive> ("GenericDrive");
    declare <KinematicControl> ("KinematicControl");
    
    parser_.addConverter <Line> ();
    parser_.addConverter <Frame> ();
    parser_.addConverter <Goal> ();
  }
  
  
  Factory & Factory::
  instance()
  {
    static Factory * instance (0);
    if ( ! instance) {
      instance = new Factory();
    }
    return *instance;
  }
  
  
  bool Factory::
  parseFile (string const &yaml_filename, ostream *erros, ostream *dbgos)
  {
    parser_.dbg = dbgos;
    if ( ! parser_.parseFile (yaml_filename)) {
      if (erros) {
	*erros << "YAML error: " << parser_.error << "\n";
      }
      return false;
    }
    return true;
  }
  
}
