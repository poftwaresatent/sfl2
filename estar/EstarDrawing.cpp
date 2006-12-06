/* 
 * Copyright (C) 2006 Roland Philippsen <roland dot philippsen at gmx dot net>
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


#include "EstarDrawing.hpp"
#include <npm/common/wrap_gl.hpp>
#include <sfl/util/numeric.hpp>
#include <sfl/gplan/GridFrame.hpp>
#include <estar/graphics.hpp>
#include <estar/util.hpp>
#include <estar/Facade.hpp>
#include <boost/shared_ptr.hpp>
#include <iostream>
//#include <cmath>

//// for boost::tie
//#include <boost/graph/adjacency_list.hpp>


#define PDEBUG PDEBUG_ERR
#define PVDEBUG PDEBUG_OFF


using namespace sfl;
using namespace boost;


EstarDrawing::
EstarDrawing(const std::string & name,
	     shared_ptr<PlanProxy> proxy,
	     what_t _what)
  : Drawing(name),
    what(_what),
    m_proxy(proxy)
{
}


void EstarDrawing::
Draw()
{
  const estar::Facade * facade(m_proxy->GetFacade());
  const sfl::GridFrame * gframe(m_proxy->GetFrame());
  if(( ! facade) || ( ! gframe))
    return;
  
  glMatrixMode(GL_MODELVIEW);
  glPushMatrix();
  glTranslated(gframe->X(), gframe->Y(), 0);
  glRotated(180 * gframe->Theta() / M_PI, 0, 0, 1);
  glScaled(gframe->Delta(), gframe->Delta(), 1);
  
  const gfx::ColorScheme * cs(gfx::ColorScheme::Get(gfx::GREY_WITH_SPECIAL));
  
  if(VALUE == what)
    gfx::draw_grid_value(*facade, cs, true);
  else
    gfx::draw_grid_meta(*facade, cs);
  
  double x0, y0, x1, y1;
  gfx::get_grid_bbox(*facade, x0, y0, x1, y1);
  glColor3d(1, 1, 0);
  glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
  glRectd(x0, y0, x1, y1);
  
  glMatrixMode(GL_MODELVIEW);
  glPopMatrix();
}
