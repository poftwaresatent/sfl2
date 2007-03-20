/* -*- mode: C++; tab-width: 2 -*- */
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


#define PDEBUG PDEBUG_ERR
#define PVDEBUG PDEBUG_OFF


using namespace sfl;
using namespace boost;
using namespace std;


EstarDrawing::
EstarDrawing(const std::string & name,
	     shared_ptr<PlanProxy> proxy,
	     what_t _what)
  : Drawing(name),
    what(_what),
    m_proxy(proxy)
{
}


EstarDrawing::
EstarDrawing(const std::string & name,
	     shared_ptr<PlanProxy> proxy,
	     what_t _what,
	     shared_ptr<gfx::ColorScheme> custom_cs)
  : Drawing(name),
    what(_what),
    m_proxy(proxy),
    m_custom_cs(custom_cs)
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
  
  const gfx::ColorScheme * cs;
  bool autoscale_value;
  if(m_custom_cs){
    cs = m_custom_cs.get();
    autoscale_value = false;
  }
  else{
    cs = gfx::ColorScheme::Get(gfx::GREY_WITH_SPECIAL);
    autoscale_value = true;
  }
  
  switch(what){
  case VALUE:
    gfx::draw_grid_value(*facade, cs, autoscale_value);
    break;
  case META:
    gfx::draw_grid_meta(*facade, cs);
    break;
  case QUEUE:
    gfx::draw_grid_queue(*facade);
    break;
  case UPWIND:
    gfx::draw_grid_upwind(*facade, 1, 0, 0, 2);
    break;
  case OBST:
    gfx::draw_grid_obstacles(*facade, 1, 0.5, 0.5);
    break;
  default:
    cerr << "ERROR in EstarDrawing::Draw(): invalid what=" << what
	 << " (expected VALUE, META, QUEUE, or UPWIND)\n";
    exit(EXIT_FAILURE);
  }
  
  double x0, y0, x1, y1;
  gfx::get_grid_bbox(*facade, x0, y0, x1, y1);
  glColor3d(1, 1, 0);
  glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
  glRectd(x0, y0, x1, y1);
  
  glMatrixMode(GL_MODELVIEW);
  glPopMatrix();
}
