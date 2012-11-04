/* 
 * Copyright (C) 2012 Roland Philippsen <roland dot philippsen at gmx dot net>
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
#include <npm/World.hpp>
#include <npm/ext/Zombie.hpp>
#include <npm/ext/expo02/Robox.hpp>
#include <npm/gfx/View.hpp>
#include <sfl/util/Line.hpp>
#include <fpplib/yaml_parser.hpp>
#include <err.h>

  
void operator >> (const YAML::Node & node, sfl::Line & ll)
{
  node[0] >> ll.p0._x;
  node[1] >> ll.p0._y;
  node[2] >> ll.p1._x;
  node[3] >> ll.p1._y;
}
  
  
void operator >> (const YAML::Node & node, npm::qhgoal_s & gg)
{
  node[0] >> gg.x;
  node[1] >> gg.y;
  node[2] >> gg.theta;
  node[3] >> gg.dr;
  node[4] >> gg.dtheta;
}
  
  
void operator >> (const YAML::Node & node, npm::qhpose_s & pp)
{
  node[0] >> pp.x;
  node[1] >> pp.y;
  node[2] >> pp.theta;
}
  
  
void operator >> (const YAML::Node & node, npm::qhwin_s & ww)
{
  node[0] >> ww.x;
  node[1] >> ww.y;
  node[2] >> ww.w;
  node[3] >> ww.h;
}


namespace npm {
  
  
  Factory::
  Factory()
  {
    declare<World>("World");
    declare<Robox>("Robox");
    declare<Zombie>("Zombie");
    declare<LidarZombie>("LidarZombie");
    declare<View>("View");
  }
  
  
  World * Factory::
  GetWorld()
  {
    fpplib::Registry<World> const *wr(findRegistry<World>());
    if ( !wr) {
      warnx (__FILE__": %s: no world registry", __func__);
      return 0;
    }
    if (wr->size() != 1) {
      warnx (__FILE__": %s: expected one world, but got %zu", __func__, wr->size());
      return 0;
    }
    return wr->at(0);
  }
  
  
  bool Factory::
  ParseFile(std::string const &yaml_filename, std::ostream *erros, std::ostream *dbgos)
  {
    fpplib::YamlParser pp(*this);
    pp.dbg = dbgos;
    pp.addConverter<sfl::Line>();
    pp.addConverter<qhgoal_s>();
    pp.addConverter<qhpose_s>();
    pp.addConverter<qhwin_s>();
    if ( !pp.parseFile(yaml_filename)) {
      if (erros) {
	*erros << "YAML error: " << pp.error << "\n";
      }
      return false;
    }
    return true;
  }
  
}
