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

#include "PlanningThreadDrawing.hpp"
#include <npm/common/wrap_gl.hpp>
#include <npm/common/View.hpp>
#include <npm/common/BBox.hpp>
#include <sfl/util/pdebug.hpp>
#include <smartsfl/PlanningThread.hpp>
#include <smartsfl/SmartPlanThread.hpp>


#ifdef NPM_DEBUG
# define PDEBUG PDEBUG_OUT
#else // ! NPM_DEBUG
# define PDEBUG PDEBUG_OFF
#endif // NPM_DEBUG
#define PVDEBUG PDEBUG_OFF


using namespace asl;
using namespace npm;


PlanningThreadDrawing::
PlanningThreadDrawing(const std::string & name,
		      const asl::PlanningThread * planning_thread)
  : Drawing(name),
    Camera(name, true),
    m_planning_thread(planning_thread)
{
}


static void set_bg_color(SmartPlanThread::status_t status)
{
  switch(status){
  case SmartPlanThread::HAVE_PLAN:   glColor3d(0,   1,   0  ); break;
  case SmartPlanThread::BUFFERING:   glColor3d(0.5, 1,   0  ); break;
  case SmartPlanThread::_PLANNING:   glColor3d(1,   0.5, 0  ); break;
  case SmartPlanThread::AT_GOAL:     glColor3d(0,   0,   1  ); break;
  case SmartPlanThread::UNREACHABLE:
  case SmartPlanThread::OUT_OF_GRID:
  case SmartPlanThread::IN_OBSTACLE: glColor3d(1,   0,   0  ); break;
  case SmartPlanThread::ERROR:       glColor3d(1,   0,   0.5); break;
  default:                           glColor3d(1,   0,   1  );
  }
}


static void set_fg_color(SmartPlanThread::status_t status)
{
  switch(status){
  case SmartPlanThread::HAVE_PLAN:   glColor3d(0.5, 0,   0  ); break;
  case SmartPlanThread::BUFFERING:   glColor3d(0.5, 0,   0.5); break;
  case SmartPlanThread::_PLANNING:   glColor3d(0.5, 0,   0.5); break;
  case SmartPlanThread::AT_GOAL:     glColor3d(0,   0.5, 0  ); break;
  case SmartPlanThread::UNREACHABLE:
  case SmartPlanThread::OUT_OF_GRID:
  case SmartPlanThread::IN_OBSTACLE: glColor3d(0,   0,   0.5); break;
  case SmartPlanThread::ERROR:       glColor3d(0,   0.5, 0.5); break;
  default:                           glColor3d(0,   0.5, 0.5);
  }
}


void PlanningThreadDrawing::
Draw()
{
  const PlanningThread::stats_t & stats(m_planning_thread->stats);
  if(stats.status.empty())
    return;
  if(1 == stats.status.size()){
    set_bg_color(stats.status[0]);
    glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
    glRectd(0, 0, 1, 1);
  }
  else{
    glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
    for(size_t ii(0); ii < stats.status.size(); ++ii){
      set_bg_color(stats.status[ii]);
      glRectd(ii    , stats.min_delta_ms,
	      ii + 1, stats.max_delta_ms);
    }
    glBegin(GL_LINE_STRIP);
    for(size_t ii(0); ii < stats.status.size(); ++ii){
      set_fg_color(stats.status[ii]);
      glVertex2d(ii + 0.5, stats.delta_ms[ii]);
    }
    glEnd();
  }
}


void PlanningThreadDrawing::
ConfigureView(npm::View & view)
{
  const PlanningThread::stats_t & stats(m_planning_thread->stats);
  view.UnlockAspectRatio();
  if(1 >= stats.status.size())
    view.SetBounds(BBox(0, 1, 0, 1), 0.1);
  else
    view.SetBounds(BBox(0, stats.status.size(), stats.min_delta_ms,
			stats.max_delta_ms), 0.1);
}
