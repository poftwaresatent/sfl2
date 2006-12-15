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
#include <estar/Grid.hpp>
#include <sfl/gplan/GridFrame.hpp>
#include <iostream>

using namespace sfl;
using namespace estar;
using namespace boost;
using namespace std;


PlanThread::
PlanThread(shared_ptr<Facade> estar,
	   const GridFrame & gframe,
	   size_t grid_xsize, size_t grid_ysize)
  : ////SimpleThread("PlanThread"),
    m_estar(estar),
    m_gframe(gframe),
    m_grid_xsize(grid_xsize),
    m_grid_ysize(grid_ysize)
    ////, m_mutex(Mutex::Create("PlanThread"))
{
  ////   if( ! m_mutex){
  ////     cerr << "ERROR in PlanThread ctor: could not allocate mutex\n";
  ////     exit(EXIT_FAILURE);
  ////   }
}


void PlanThread::
Step()
{
  ////  Mutex::sentry ss(m_mutex);
  if( ! m_estar->HaveWork())
    return;
  m_estar->ComputeOne();
}


PlanThread::status_t PlanThread::
GetStatus(double robot_global_x, double robot_global_y)
{
  const GridFrame::index_t
    idx(m_gframe.GlobalIndex(robot_global_x, robot_global_y));
  
  if((idx.v0 >= m_grid_xsize) || (idx.v1 >= m_grid_ysize))
    return OUT_OF_GRID;
  
  const double meta(m_estar->GetMeta(idx.v0, idx.v1));
  if(meta == m_estar->GetObstacleMeta())
    return IN_OBSTACLE;
  
  const Algorithm & algo(m_estar->GetAlgorithm());
  const flag_map_t & flag_map(algo.GetFlagMap());
  const vertex_t rnode(m_estar->GetGrid().GetVertex(idx.v0, idx.v1));
  const flag_t flag(get(flag_map, rnode));
  if(flag & GOAL){
    ////    cerr << "AT_GOAL\n";
    return AT_GOAL;
  }
  if(flag & OPEN){
    ////    cerr << "OPEN -> PLANNING\n";
    return PLANNING;
  }
  
  //const double lcvalplus(get(m_estar->GetAlgorithm().GetValueMap(), lcnode));
  //const double lcvalminus(m_estar->GetAlgorithm().GetLastComputedValue());
  //const double lcval(minval(lcvalplus, lcvalminus));
  const double lpkey(algo.GetLastPoppedKey());
  const double value(m_estar->GetValue(idx.v0, idx.v1));
  
  //cerr << "lcval = min(" << lcvalminus << " " << lcvalplus << ") = " << lcval
  ////  cerr << "lpkey = " << lpkey << "  value = " << value;
  //  if((lcval <= value) || (lcval == infinity)){
  if(lpkey <= value){
    if(m_estar->HaveWork()){
      ////      cerr << " PLANNING\n";
      return PLANNING;
    }
    ////    cerr << " UNREACHABLE\n";    
    return UNREACHABLE;
  }
  ////  cerr << " HAVE_PLAN\n";    
  return HAVE_PLAN;
}
