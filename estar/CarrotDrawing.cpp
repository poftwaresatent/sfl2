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


#include "CarrotDrawing.hpp"
#include "Esbot.hpp"
#include "PNF.hpp"
#include <npm/common/wrap_gl.hpp>
#include <npm/common/wrap_glu.hpp>
#include <sfl/util/numeric.hpp>
#include <sfl/gplan/GridFrame.hpp>
#include <estar/util.hpp>
#include <math.h>

// debugging
#include <estar/util.hpp>
#define PDEBUG PDEBUG_OFF
#define PVDEBUG PDEBUG_OFF


using namespace sfl;
using namespace estar;
using namespace boost;


CarrotDrawing::
CarrotDrawing(const std::string & name,
	      Esbot * bot,
	      bool _global_mode,
	      bool _full_trace,
	      size_t _gradplot_frequency)
  : Drawing(name),
    global_mode(_global_mode),
    full_trace(_full_trace),
    gradplot_frequency(_gradplot_frequency),
    m_bot(bot)
{
}
  

void CarrotDrawing::
Draw()
{
  boost::shared_ptr<PNF> pnf(m_bot->GetPNF());
  if( ! pnf){
    PDEBUG("carrot: no PNF in bot\n");
    return;
  }
  
  shared_ptr<carrot_trace> trace;
  if(full_trace){
    PDEBUG("carrot: full trace\n");
    trace = m_bot->ComputeFullCarrot();
  }
  else{
    PDEBUG("carrot: partial trace\n");    
    trace = m_bot->GetCarrotTrace();
  }
  if((!trace) || (trace->empty())){
    PDEBUG("carrot: invalid or empty trace\n");    
    return;
  }

  double carx, cary;
  if(full_trace){
    const carrot_item ci(m_bot->GetCarrotTrace()->back());
    carx = ci.cx;
    cary = ci.cy;
  }
  else{
    carx = trace->back().cx;
    cary = trace->back().cy;
  }
  
  shared_ptr<const GridFrame> gframe(pnf->GetGridFrame());
  Frame lframe(m_bot->GetPose());
  gframe->From(lframe);
  
  glMatrixMode(GL_MODELVIEW);
  glPushMatrix();
  if(global_mode){
    glTranslated(gframe->X(), gframe->Y(), 0);
    glRotated(180 * gframe->Theta() / M_PI, 0, 0, 1);
  }
  else{
    glTranslated(lframe.X(), lframe.Y(), 0);
    glRotated(180 * lframe.Theta() / M_PI, 0, 0, 1);    
  }
  
  const double startval(trace->front().value);
  double deltav(absval(startval - trace->back().value));
  if(deltav <= epsilon)
    deltav = startval;
  if(0 == gradplot_frequency){
    glBegin(GL_LINE_STRIP);
    for(size_t ii(0); ii < trace->size(); ++ii){
      const double blue((startval - (*trace)[ii].value) / deltav);
      if((*trace)[ii].degenerate)
	glColor3d(1, 0.5, blue);
      else
	glColor3d(0.5, 1, blue);
      glVertex2d((*trace)[ii].cx, (*trace)[ii].cy);
    }
    glEnd();
  }
  else{
    glPointSize(3);
    glBegin(GL_POINTS);
    for(size_t ii(0); ii < trace->size(); ii += gradplot_frequency){
      const double blue((startval - (*trace)[ii].value) / deltav);
      if((*trace)[ii].degenerate)
	glColor3d(1, 0.5, blue);
      else
	glColor3d(0.5, 1, blue);
      glVertex2d((*trace)[ii].cx, (*trace)[ii].cy);
    }
    glEnd();
    glPointSize(1);
    glBegin(GL_LINES);
    for(size_t ii(0); ii < trace->size(); ii += gradplot_frequency){
      const double blue((startval - (*trace)[ii].value) / deltav);
      if((*trace)[ii].degenerate)
	glColor3d(1, 0.5, blue);
      else
	glColor3d(0.5, 1, blue);
      glVertex2d((*trace)[ii].cx, (*trace)[ii].cy);
      glVertex2d((*trace)[ii].cx + (*trace)[ii].gradx,
		 (*trace)[ii].cy + (*trace)[ii].grady);
    }
    glEnd();
  }
  
  glColor3d(1, 1, 0);
  glPushMatrix();
  glTranslated(carx, cary, 0);
  gluDisk(wrap_glu_quadric_instance(), 0.2, 0.2, 36, 1);
  glPopMatrix();
  
  glMatrixMode(GL_MODELVIEW);
  glPopMatrix();
}
