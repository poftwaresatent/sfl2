/* 
 * Copyright (C) 2006
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


#include "PlanThread.hpp"
#include <estar/Facade.hpp>
#include <estar/Queue.hpp>
#include <estar/Algorithm.hpp>
//#include <estar/dump.hpp>
#include <sfl/gplan/GridFrame.hpp>
#include <iostream>

using namespace sfl;
using namespace estar;
using namespace boost;
using namespace std;


PlanThread::
PlanThread(shared_ptr<Facade> estar,
	   shared_ptr<const sfl::GridFrame> gframe,
	   size_t grid_xsize, size_t grid_ysize)
  : SimpleThread("PlanThread"),
    m_estar(estar),
    m_gframe(gframe),
    m_grid_xsize(grid_xsize),
    m_grid_ysize(grid_ysize),
    m_mutex(Mutex::Create("PlanThread"))
{
  if( ! m_mutex){
    cerr << "ERROR in PlanThread ctor: could not allocate mutex\n";
    exit(EXIT_FAILURE);
  }
}


void PlanThread::
Step()
{
  Mutex::sentry ss(m_mutex);
  if( ! m_estar->HaveWork())
    return;
  m_estar->ComputeOne();
  //  dump_queue(*m_estar, 0, stdout);
}


int PlanThread::
GetStatus(double robot_global_x, double robot_global_y)
{
  const GridFrame::index_t
    idx(m_gframe->GlobalIndex(robot_global_x, robot_global_y));

  //   PDEBUG("pos %g   %g   idx %zu %zu\n",
  // 	 robot_global_x, robot_global_y, idx.v0, idx.v1);
  
  if((idx.v0 >= m_grid_xsize) || (idx.v1 >= m_grid_ysize))
    return -1;
  
  Mutex::sentry ss(m_mutex);
  
  const double rmeta(m_estar->GetMeta(idx.v0, idx.v1));
  if(rmeta == m_estar->GetObstacleMeta())
    return -2;
  
  const double rval(m_estar->GetValue(idx.v0, idx.v1));
  if( ! m_estar->HaveWork()){
    //    PDEBUG_OUT("no work left to do\n");
    if(rval < infinity)
      return 0;
    return 2;
  }
  
  const queue_t queue(m_estar->GetAlgorithm().GetQueue().Get());
  if(queue.empty()){
    cerr << "BUG in PlanThread::GetStatus():"
	 << " m_estar->HaveWork() but queue.empty()!\n";
    exit(EXIT_FAILURE);
  }
  queue_t::const_iterator qt(queue.begin());
  if(qt->first <= rval){
    //    PDEBUG_OUT("robot is downwind of wavefront\n");
    return 1;
  }
  return 0;
}
