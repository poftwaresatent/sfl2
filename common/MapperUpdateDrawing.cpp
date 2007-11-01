/* -*- mode: C++; tab-width: 2 -*- */
/* 
 * Copyright (C) 2007
 * Swiss Federal Institute of Technology, Zurich. All rights reserved.
 * 
 * Developed at the Autonomous Systems Lab.
 * Visit our homepage at http://www.asl.ethz.ch/
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

#include "MapperUpdateDrawing.hpp"
#include "wrap_gl.hpp"
#include "Manager.hpp"
#include <sfl/gplan/Mapper2d.hpp>
#include <sfl/util/pdebug.hpp>
#include <cmath>


#ifdef NPM_DEBUG
# define PDEBUG PDEBUG_OUT
#else // ! NPM_DEBUG
# define PDEBUG PDEBUG_OFF
#endif // NPM_DEBUG
#define PVDEBUG PDEBUG_OFF


using namespace sfl;


namespace npm {
  
  
  MapperUpdateDrawing::
  MapperUpdateDrawing(const std::string & name,
		      boost::shared_ptr<const sfl::Mapper2d> mapper)
    : Drawing(name,
							"most recent changes of a sweeping-update sfl::Mapper2d",
							Instance<UniqueManager<Drawing> >()),
      m_mapper(mapper)
  {
  }
  

  void MapperUpdateDrawing::
  Draw()
  {
    const Mapper2d::link_t & freespace(m_mapper->GetFreespaceBuffer());
    const Mapper2d::link_t & obstacle(m_mapper->GetObstacleBuffer());
    
    if(freespace.empty() && obstacle.empty())
      return;
    
    const GridFrame & gframe(m_mapper->GetGridFrame());
    
    glMatrixMode(GL_MODELVIEW);
    glPushMatrix();
    glTranslated(gframe.X(), gframe.Y(), 0);
    glRotated(180 * gframe.Theta() / M_PI, 0, 0, 1);
    glScaled(gframe.Delta(), gframe.Delta(), 1);
    
    glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
    
    glColor3d(0, 1, 0.5);
    for(Mapper2d::link_t::const_iterator ifree(freespace.begin());
				ifree != freespace.end(); ++ifree){
      PDEBUG("free %ul %ul\n", ifree->v0, ifree->v1);
      glRectd(ifree->v0 - 0.3, ifree->v1 - 0.3,
							ifree->v0 + 0.3, ifree->v1 + 0.3);
    }
    
    glColor3d(1, 0, 0.5);
    for(Mapper2d::link_t::const_iterator iobst(obstacle.begin());
				iobst != obstacle.end(); ++iobst){
      PDEBUG("obst %ul %ul\n", iobst->v0, iobst->v1);
      glRectd(iobst->v0 - 0.4, iobst->v1 - 0.4,
							iobst->v0 + 0.4, iobst->v1 + 0.4);
    }
    
    glMatrixMode(GL_MODELVIEW);
    glPopMatrix();    
  }

}
