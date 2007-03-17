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


#include "PathDrawing.hpp"
#include "Smart.hpp"
#include <npm/common/wrap_gl.hpp>
#include <npm/common/wrap_glu.hpp>
#include <sfl/util/numeric.hpp>
#include <sfl/gplan/GridFrame.hpp>
#include <asl/path_tracking.hpp>
#include <math.h>

// debugging
#include <estar/util.hpp>
#define PDEBUG PDEBUG_OFF
#define PVDEBUG PDEBUG_OFF


using namespace sfl;
using namespace estar;
using namespace boost;


PathDrawing::
PathDrawing(const std::string & name,
	    const Smart * smart,
	    size_t _gradplot_frequency)
  : Drawing(name),
    gradplot_frequency(_gradplot_frequency),
    m_smart(smart)
{
}


void PathDrawing::
Draw()
{
  const asl::path_t * path(m_smart->GetPath());
  if(( ! path) || (path->empty())){
    PDEBUG("carrot: invalid or empty path\n");    
    return;
  }
  
  const double startval(path->front().value);
  double deltav(absval(startval - path->back().value));
  if(deltav <= epsilon)
    deltav = startval;
  
  if(0 == gradplot_frequency){
    glBegin(GL_LINE_STRIP);
    for(size_t ii(0); ii < path->size(); ++ii){
      const double blue((startval - (*path)[ii].value) / deltav);
      if((*path)[ii].degenerate)
	glColor3d(1, 0.5, blue);
      else
	glColor3d(0.5, 1, blue);
      glVertex2d((*path)[ii].point.v0, (*path)[ii].point.v1);
    }
    glEnd();
  }
  else{
    glPointSize(3);
    glBegin(GL_POINTS);
    for(size_t ii(0); ii < path->size(); ii += gradplot_frequency){
      const double blue((startval - (*path)[ii].value) / deltav);
      if((*path)[ii].degenerate)
	glColor3d(1, 0.5, blue);
      else
	glColor3d(0.5, 1, blue);
      glVertex2d((*path)[ii].point.v0, (*path)[ii].point.v1);
    }
    glEnd();
    glPointSize(1);
    glBegin(GL_LINES);
    for(size_t ii(0); ii < path->size(); ii += gradplot_frequency){
      const double blue((startval - (*path)[ii].value) / deltav);
      if((*path)[ii].degenerate)
	glColor3d(1, 0.5, blue);
      else
	glColor3d(0.5, 1, blue);
      glVertex2d((*path)[ii].point.v0, (*path)[ii].point.v1);
      glVertex2d((*path)[ii].point.v0 + (*path)[ii].gradient.v0,
		 (*path)[ii].point.v1 + (*path)[ii].gradient.v1);
    }
    glEnd();
  }
  
  double carx(path->back().point.v0);
  double cary(path->back().point.v1);
  glColor3d(1, 1, 0);
  glPushMatrix();
  glTranslated(carx, cary, 0);
  gluDisk(wrap_glu_quadric_instance(), 0.2, 0.2, 36, 1);
  glPopMatrix();
}
