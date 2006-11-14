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


#include "PNF.hpp"
#include <sfl/util/numeric.hpp>
#include <sfl/gplan/GridFrame.hpp>
#include <pnf/Flow.hpp>
#include <pnf/BufferZone.hpp>
#include <pnf/PNFRiskMap.hpp>
#include <iostream>
#include <cmath>

// for deletion of smart pointers containing forwardly declared types:
#include <estar/Algorithm.hpp>
#include <estar/Facade.hpp>
#include <estar/Grid.hpp>
#include <estar/Kernel.hpp>

// debugging
#include <estar/util.hpp>
#define PDEBUG PDEBUG_ERR
#define PVDEBUG PDEBUG_OFF


using sfl::GridFrame;
using sfl::Frame;
using sfl::absval;
using pnf::BufferZone;
using pnf::Flow;
using std::cerr;


static void * run(void * arg);


namespace local {
  
  typedef enum { IDLE, RUNNING, WAIT, QUIT } state_t;
  
  class poster
  {
  public:
    pthread_t thread_id;
    pthread_mutex_t mutex;
    state_t state;
    PNF::step_t step;
    pnf::Flow * flow;
    const double robot_r;

    poster(pnf::Flow * _flow, double _robot_r)
      : state(WAIT), step(PNF::NONE), flow(_flow), robot_r(_robot_r)
    {
      pthread_mutexattr_t attr;
      pthread_mutexattr_init(&attr);
      pthread_mutexattr_settype(&attr, PTHREAD_MUTEX_ERRORCHECK);
      pthread_mutex_init(&mutex, &attr);
      pthread_mutexattr_destroy(&attr);
      switch(pthread_create(&thread_id, 0, run, this)){
      case EAGAIN:
	cerr << __func__ << "(): insufficient resources for thread.\n";
	exit(EXIT_FAILURE);
      case EINVAL:
	cerr << __func__ << "(): BUG [invalid attr for pthread_create()]\n";
	exit(EXIT_FAILURE);
      }
    }
    
    ~poster()
    {
      if(EINVAL == pthread_mutex_lock(&mutex)){
	PDEBUG("WARNING pthread_mutex_lock() failed ==> stale poster\n");
	return;
      }
      state = QUIT;
      if(EINVAL == pthread_mutex_unlock(&mutex)){
	PDEBUG("WARNING pthread_mutex_unlock() failed ==> stale poster\n");
	return;
      }
      pthread_join(thread_id, 0);
      pthread_mutex_destroy(&mutex);
    }
  };
  
}


PNF::
PNF(double _robot_x, double _robot_y,
    double _robot_r, double _robot_v,
    double _goal_x, double _goal_y, double _goal_r,
    double _grid_width, size_t _grid_wdim)
  : robot_x(_robot_x),
    robot_y(_robot_y),
    robot_r(_robot_r),
    robot_v(_robot_v),
    goal_x(_goal_x),
    goal_y(_goal_y),
    goal_r(_goal_r),
    grid_width(_grid_width),
    grid_wdim(_grid_wdim),
    resolution(grid_width / grid_wdim)
{
  const double dx(goal_x - robot_x);
  const double dy(goal_y - robot_y);
  const double width_offset(resolution * (grid_wdim - 1) / 2.0);
  double xm_frame(-width_offset);
  double ym_frame(-width_offset);
  const Frame frame(robot_x, robot_y, atan2(dy, dx));
  frame.To(xm_frame, ym_frame);
  
  m_frame.reset(new GridFrame(xm_frame, ym_frame, frame.Theta(), resolution));
  
  const size_t
    xdim(static_cast<size_t>(ceil((sqrt(dx*dx+dy*dy)+grid_width)/resolution)));
  const bool perform_convolution(false);
  const bool alternate_worst_case(false);
  m_flow.reset(Flow::Create(xdim, grid_wdim, resolution,
			    perform_convolution, alternate_worst_case));
  
  // very important to use LOCAL grid coordinates
  m_frame->From(_robot_x, _robot_y);
  if( ! m_flow->SetRobot(_robot_x, _robot_y, robot_r, robot_v)){
    cerr << "m_flow->SetRobot(" << _robot_x << ", " << _robot_y << ", "
	 << robot_r << ", " << robot_v << ") failed\n";
    exit(EXIT_FAILURE);
  }
  PVDEBUG("SetRobot(%g   %g   %g   %g) on flow %lu\n",
	  _robot_x, _robot_y, robot_r, robot_v, m_flow.get());

  m_frame->From(_goal_x, _goal_y);
  if( ! m_flow->SetGoal(_goal_x, _goal_y, goal_r)){
    cerr << "m_flow->SetGoal(" << _goal_x << ", " << _goal_y << ", "
	 << goal_r << ") failed\n";
    exit(EXIT_FAILURE);
  }
  PVDEBUG("SetGoal(%g   %g   %g) on flow %lu\n",
	  _goal_x, _goal_y, goal_r, m_flow.get());
  
  m_poster.reset(new local::poster(m_flow.get(), robot_r));
}


bool PNF::
AddStaticObject(double globx, double globy)
{
  PVDEBUG("%g   %g\n", globx, globy);
  Wait();
  const GridFrame::index_t
    idx(m_frame->GlobalIndex(GridFrame::position_t(globx, globy)));
  if((idx.v0 < 0) || (idx.v1 < 0)
     || (static_cast<size_t>(idx.v0) >= m_flow->xsize)
     || (static_cast<size_t>(idx.v1) >= m_flow->ysize))
    return false;
  m_flow->AddStaticObject(static_cast<size_t>(idx.v0),
			  static_cast<size_t>(idx.v1));
  return true;
}


bool PNF::
SetDynamicObject(size_t id, double globx, double globy,
		 double r, double v)
{
  PVDEBUG("%lu   %g   %g   %g   %g\n", id, globx, globy, r, v);
  Wait();
  m_frame->From(globx, globy);
  return m_flow->SetDynamicObject(id, globx, globy, r, v);
}


bool PNF::
RemoveStaticObject(double globx, double globy)
{
  PVDEBUG("%g   %g\n", globx, globy);
  Wait();  
  const GridFrame::index_t
    idx(m_frame->GlobalIndex(GridFrame::position_t(globx, globy)));
  if((idx.v0 < 0) || (idx.v1 < 0)
     || (static_cast<size_t>(idx.v0) >= m_flow->xsize)
     || (static_cast<size_t>(idx.v1) >= m_flow->ysize))
    return false;
  m_flow->RemoveStaticObject(static_cast<size_t>(idx.v0),
			     static_cast<size_t>(idx.v1));
  return true;
}


void PNF::
RemoveDynamicObject(size_t id)
{
  PVDEBUG("%lu\n", id);
  Wait();
  m_flow->RemoveDynamicObject(id);
}


void PNF::
StartPlanning()
{
  PVDEBUG("m_poster->flow is instance %lu\n", m_poster->flow);
  Wait();
  if(EINVAL == pthread_mutex_lock(&m_poster->mutex)){
    cerr << __func__ << "(): pthread_mutex_lock() error\n";
    exit(EXIT_FAILURE);
  }
  m_poster->state = local::RUNNING;
  m_poster->step = NONE;
  if(EINVAL == pthread_mutex_unlock(&m_poster->mutex)){
    cerr << __func__ << "(): pthread_mutex_unlock() error\n";
    exit(EXIT_FAILURE);
  }
}


PNF::step_t PNF::
GetStep() const
{
  if(EINVAL == pthread_mutex_lock(&m_poster->mutex)){
    cerr << __func__ << "(): pthread_mutex_lock() error\n";
    exit(EXIT_FAILURE);
  }
  step_t result(NONE);
  if(local::WAIT != m_poster->state)
    result = m_poster->step;
  if(EINVAL == pthread_mutex_unlock(&m_poster->mutex)){
    cerr << __func__ << "(): pthread_mutex_unlock() error\n";
    exit(EXIT_FAILURE);
  }
  return result;
}


void PNF::
Wait()
{
  if(EINVAL == pthread_mutex_lock(&m_poster->mutex)){
    cerr << __func__ << "(): pthread_mutex_lock() error\n";
    exit(EXIT_FAILURE);
  }
  m_poster->state = local::WAIT;
  m_poster->step = NONE;
  if(EINVAL == pthread_mutex_unlock(&m_poster->mutex)){
    cerr << __func__ << "(): pthread_mutex_unlock() error\n";
    exit(EXIT_FAILURE);
  }
}


void * run(void * arg)
{
  static const pnf::Sigma riskmap(0.95, 1.5); // TO DO: Magic numbers stink!
  local::poster * poster(reinterpret_cast<local::poster*>(arg));
  bool quit(false);
  local::state_t prevstate(static_cast<local::state_t>(-1));
  while(!quit){
    if(EINVAL == pthread_mutex_lock(&poster->mutex)){
      cerr << __func__ << "(): pthread_mutex_lock() error\n";
      exit(EXIT_FAILURE);
    }
    
    if(local::WAIT == poster->state){
      if(prevstate != poster->state)
	PDEBUG("0x%08X waiting\n", poster->thread_id);
      prevstate = poster->state;
      // do nothing
    }
    else if(local::QUIT == poster->state){
      PDEBUG("0x%08X quit\n", poster->thread_id);
      quit = true;
    }
    else{
      PVDEBUG("poster->flow is instance %lu\n", poster->flow);
      
      if( ! poster->flow->HaveEnvdist()){
	PDEBUG("PropagateEnvdist()...\n");
	poster->flow->PropagateEnvdist(false);
	if( ! poster->flow->HaveEnvdist()){
	  PDEBUG("PropagateEnvdist() failed\n");
	  poster->step = PNF::NONE;
	}
	else{
	  PDEBUG("PropagateEnvdist() succeeded\n");
	  poster->flow->MapEnvdist();
	  poster->step = PNF::ENVDIST;
	}
      }
      
      else if( ! poster->flow->HaveAllObjdist()){
	// could use PropagateObjdist(size_t id) for granularity...
	PVDEBUG("PropagateAllObjdist()...\n");
	poster->flow->PropagateAllObjdist();
	if( ! poster->flow->HaveAllObjdist()){
	  PDEBUG("PropagateAllObjdist() failed\n");
	  poster->step = PNF::ENVDIST;
	}
	else{
	  PDEBUG("PropagateAllObjdist() succeeded\n");
	  poster->step = PNF::OBJDIST;
	}
      }
      
      else if( ! poster->flow->HaveRobdist()){
	PVDEBUG("PropagateRobdist()...\n");
	poster->flow->PropagateRobdist();
	if( ! poster->flow->HaveRobdist()){
	  PDEBUG("PropagateRobdist() failed\n");
	  poster->step = PNF::OBJDIST;
	}
	else{
	  PDEBUG("PropagateRobdist() succeeded\n");
	  const double static_buffer_factor(1);	// multiplies robot_radius (?)
	  const double static_buffer_degree(2);
	  poster->flow->ComputeAllCooc(static_buffer_factor,
				       static_buffer_degree);
	  poster->flow->ComputeRisk(riskmap);
	  poster->step = PNF::ROBDIST;
	}
      }
      
      else if( ! poster->flow->HavePNF()){
	PVDEBUG("PropagatePNF()...\n");
	poster->flow->PropagatePNF();
	if( ! poster->flow->HavePNF()){
	  PDEBUG("PropagatePNF() failed\n");
	  poster->step = PNF::ROBDIST;
	}
	else{
	  PDEBUG("PropagatePNF() succeeded\n");
	  poster->step = PNF::DONE;
	}
      }
      
      if(PNF::DONE == poster->step)
	poster->state = local::IDLE;
      else
	poster->state = local::RUNNING;

      if(prevstate != poster->state)
	PDEBUG("0x%08X %s\n", poster->thread_id,
	       (local::IDLE == poster->state) ? "idle" : "running");
      prevstate = poster->state;
    }
    
    if(EINVAL == pthread_mutex_unlock(&poster->mutex)){
      cerr << __func__ << "(): pthread_mutex_unlock() error\n";
      exit(EXIT_FAILURE);
    }
  }
  return 0;
}


static bool do_add_static_object(Flow * flow, double xx, double yy)
{
  const size_t ix(static_cast<size_t>(rint(xx)));
  const size_t iy(static_cast<size_t>(rint(yy)));
  PVDEBUG("%g   %g   %ul   %ul\n", xx, yy, ix, iy);
  if((ix >= flow->xsize) || (iy >= flow->ysize))
    return false;
  flow->AddStaticObject(ix, iy);
  return true;
}


/** \note Digital Differential Analyzer style algorithm */
bool PNF::
AddStaticLine(double x0, double y0,
	      double x1, double y1)
{
  PVDEBUG("global: %g   %g   %g   %g\n", x0, y0, x1, y1);
  Wait();
  
  m_frame->From(x0, y0);
  m_frame->From(x1, y1);
  x0 /= resolution;
  y0 /= resolution;
  x1 /= resolution;
  y1 /= resolution;
  PVDEBUG("local: %g   %g   %g   %g\n", x0, y0, x1, y1);
  PVDEBUG("gridsize: %ul   %ul\n", m_flow->xsize, m_flow->ysize);
  
  Flow * flow(m_flow.get());
  bool ok(false);
  if(do_add_static_object(flow, x0, y0))
    ok = true;
  
  double dx(x1 - x0);
  double dy(y1 - y0);
  if(absval(dx) > absval(dy)){
    double slope(dy / dx);
    dx = (dx < 0) ? -1 : 1;
    slope *= dx;
    while(absval(x0 - x1) >= 0.5){
      x0 += dx;
      y0 += slope;
      if(do_add_static_object(flow, x0, y0))
	ok = true;
    }
  }
  else{
    double slope(dx / dy);
    dy = (dy < 0) ? -1 : 1;
    slope *= dy;
    while(absval(y0 - y1) >= 0.5){
      y0 += dy;
      x0 += slope;
      if(do_add_static_object(flow, x0, y0))
	ok = true;
    }
  }
  
  return ok;
}
