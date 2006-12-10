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

#include "Smart.hpp"
#include "PlanThread.hpp"
#include <sfl/api/Goal.hpp>
#include <sfl/api/Scanner.hpp>
#include <sfl/util/numeric.hpp>
#include <sfl/gplan/TraversabilityMap.hpp>
#include <sfl/gplan/GridFrame.hpp>
#include <estar/Facade.hpp>
#include <estar/Algorithm.hpp>	// for delete in shared_ptr through Facade
#include <estar/Kernel.hpp>	// for delete in shared_ptr through Facade
#include <estar/Region.hpp>
#include <estar/Grid.hpp>
#include <estar/dump.hpp>	// dbg
#include <npm/robox/expoparams.hpp>
#include <npm/common/Lidar.hpp>
#include <npm/common/HAL.hpp>
#include <npm/common/RobotServer.hpp>
#include <npm/common/GoalInstanceDrawing.hpp>
#include <npm/common/CheatSheet.hpp>
#include <npm/estar/EstarDrawing.hpp>
#include <iostream>
#include <math.h>


#ifdef NPM_DEBUG
# define PDEBUG PDEBUG_OUT
#else // ! NPM_DEBUG
# define PDEBUG PDEBUG_OFF
#endif // NPM_DEBUG


using namespace npm;
using namespace sfl;
using namespace estar;
using namespace boost;
using namespace std;


class SmartPlanProxy: public PlanProxy {
public:
  SmartPlanProxy(Smart * smart): m_smart(smart) {}
  virtual const estar::Facade * GetFacade() { return m_smart->m_estar.get(); }
  virtual const sfl::GridFrame * GetFrame() { return m_smart->m_gframe.get(); }
  Smart * m_smart;
};


Smart::
Smart(shared_ptr<RobotDescriptor> descriptor, const World & world)
  : RobotClient(descriptor, world, true),
    m_goal(new Goal()),
    m_cheat(new CheatSheet(&world, GetServer())),
    m_replan_request(false)
{
  expoparams params(descriptor);
  m_nscans = params.front_nscans;
  m_sick_channel = params.front_channel;
  m_wheelbase = params.model_wheelbase;
  m_wheelradius = params.model_wheelradius;
  m_axlewidth = params.model_axlewidth;

  m_sick = DefineLidar(Frame(params.front_mount_x,
                             params.front_mount_y,
                             params.front_mount_theta),
                       params.front_nscans,
                       params.front_rhomax,
                       params.front_phi0,
                       params.front_phirange,
                       params.front_channel)->GetScanner();

  DefineBicycleDrive(m_wheelbase, m_wheelradius, m_axlewidth);

  AddLine(Line(-m_wheelradius, -m_axlewidth/2 -m_wheelradius,
	       -m_wheelradius,  m_axlewidth/2 +m_wheelradius));
  AddLine(Line(m_wheelbase +m_wheelradius, -m_axlewidth/2 -m_wheelradius,
	       m_wheelbase +m_wheelradius,  m_axlewidth/2 +m_wheelradius));
  AddLine(Line(-m_wheelradius, -m_axlewidth/2 -m_wheelradius,
	       m_wheelbase +m_wheelradius, -m_axlewidth/2 -m_wheelradius));
  AddLine(Line(-m_wheelradius,  m_axlewidth/2 +m_wheelradius,
	       m_wheelbase +m_wheelradius,  m_axlewidth/2 +m_wheelradius));
  
  AddDrawing(new GoalInstanceDrawing(descriptor->name + "_goaldrawing",
				     *m_goal));
  
  shared_ptr<SmartPlanProxy> proxy(new SmartPlanProxy(this));
  AddDrawing(new EstarDrawing(descriptor->name + "_estar_meta",
 			      proxy, EstarDrawing::META));
  AddDrawing(new EstarDrawing(descriptor->name + "_estar_value",
 			      proxy, EstarDrawing::VALUE));
  AddDrawing(new EstarDrawing(descriptor->name + "_estar_queue",
 			      proxy, EstarDrawing::QUEUE));
}


// something's wrong with sfl::SimpleThread
#define DISABLE_THREADS

void Smart::
PrepareAction(double timestep)
{
  m_sick->Update();
  
  if( ! m_travmap)
    m_travmap = m_cheat->GetTravmap();
  if( ! m_travmap){
    cerr << "ERROR: Smart needs a traversability map.\n";
    exit(EXIT_FAILURE);
  }
  if( ! m_gframe)
    m_gframe.reset(new GridFrame(m_travmap->gframe));
  
  if(m_replan_request){
    m_replan_request = false;
    
#ifndef DISABLE_THREADS
    if(m_plan_thread)
      m_plan_thread->Stop();
#endif // ! DISABLE_THREADS
    
    shared_ptr<const array2d<int> > data(m_travmap->data);
    if( ! data){
      cerr << "ERROR: Invalid traversability map (no data).\n";
      exit(EXIT_FAILURE);
    }
    
    if( ! m_estar){
      m_estar.reset(Facade::CreateDefault(data->xsize, data->ysize,
					  m_gframe->Delta()));
      
      const double scale(1.0 / (m_travmap->obstacle - m_travmap->freespace));
      for(size_t ix(0); ix < data->xsize; ++ix)
	for(size_t iy(0); iy < data->ysize; ++iy){
	  const double meta(scale * (m_travmap->obstacle - (*data)[ix][iy]));
	  m_estar->InitMeta(ix, iy, sfl::boundval(0.0, meta, 1.0));
	}
    }
    else
      m_estar->RemoveAllGoals();
    
//     obstacle_draw_callback cb(m_estar.get());
//     m_cheat->UpdateLines();
//     for(size_t il(0); il < m_cheat->line.size(); ++il)
//       m_gframe->DrawGlobalLine(m_cheat->line[il].x0, m_cheat->line[il].y0,
// 			    m_cheat->line[il].x1, m_cheat->line[il].y1,
// 			    data->xsize, data->ysize, cb);    
//     m_cheat->UpdateDynobjs();
//     for(size_t ir(0); ir < m_cheat->dynobj.size(); ++ir)
//       m_gframe->DrawGlobalDisk(m_cheat->dynobj[ir].x,
// 			    m_cheat->dynobj[ir].y,
// 			    m_cheat->dynobj[ir].r,
// 			    data->xsize, data->ysize, cb);
    
    double goalx(m_goal->X());
    double goaly(m_goal->Y());
    m_gframe->From(goalx, goaly);
    m_goalregion.reset(new Region(m_goal->Dr(), m_gframe->Delta(),
				  goalx, goaly, data->xsize, data->ysize));
    if(m_goalregion->GetArea().empty()){
      PDEBUG("ERROR: empty goal area, extend Smart::PrepareAction()!\n");
      exit(EXIT_FAILURE);
    }
    
    ////    m_estar->AddGoal(*m_goalregion);
    for(Region::indexlist_t::const_iterator
	  in(m_goalregion->GetArea().begin());
	in != m_goalregion->GetArea().end(); ++in)
      m_estar->AddGoal(in->x, in->y, in->r);
    
//     cout << "==================================================\n"
// 	 << "queue right after adding the goal region:\n";
//     dump_queue(*m_estar, 0, stdout);
//     cout << "==================================================\n";
    
    if( ! m_plan_thread)
      m_plan_thread.reset(new PlanThread(m_estar, m_gframe,
					 data->xsize, data->ysize));

#ifndef DISABLE_THREADS
    if( ! m_plan_thread->Start(100000)){
      cerr << "ERROR in Smart::PrepareAction():"
	   << " m_plan_thread->Start(100000) failed\n";
      exit(EXIT_FAILURE);
    }
#endif // ! DISABLE_THREADS
  }
  
  if( ! m_plan_thread){
    PDEBUG("WARNING: no plan thread late in Smart::PrepareAction()!\n");
    GetHAL()->speed_set(0, 0);
    return;
  }
  
#ifdef DISABLE_THREADS
  m_plan_thread->Step();  
#endif // DISABLE_THREADS
  
  //// const Frame & pose(GetServer()->GetTruePose());
  const Frame pose(GetServer()->GetTruePose());
  
  const int plan_status(m_plan_thread->GetStatus(pose.X(), pose.Y()));
  switch(plan_status){
  case 0:		  // have plan: do the stuff after this switch
    break;
  case 1:
    PDEBUG("wave hasn't crossed robot yet\n");
    GetHAL()->speed_set(0, 0);
    return;
  case 2:
    PDEBUG_ERR("ERROR: no path to goal\n");
    exit(EXIT_FAILURE);
  case -1:
    PDEBUG_ERR("ERROR: robot is outside grid\n");
    exit(EXIT_FAILURE);
  case -2:
    PDEBUG_ERR("ERROR: robot is inside an obstacle\n");
    exit(EXIT_FAILURE);
  default:
    PDEBUG_ERR("BUG: unhandled result %d of m_plan_thread->GetStatus()\n",
	       plan_status);
    exit(EXIT_FAILURE);
  }
  
  const GridFrame::index_t idx(m_gframe->GlobalIndex(pose.X(), pose.Y()));
  double gradx, grady;
  m_estar->GetGrid().ComputeGradient(idx.v0, idx.v1, gradx, grady);
  gradx = -gradx;		// follow the NEGATIVE gradient
  grady = -grady;
  
  m_gframe->RotateTo(gradx, grady);
  pose.RotateFrom(gradx, grady);
  const double phimax(M_PI / 2.1);
  const double dphi(sfl::boundval(-phimax, atan2(grady, gradx), phimax));
  const double sdmin(0.05);
  const double sdmax(0.8);
  const double scale((phimax - sfl::absval(dphi)) / phimax);
  const double sd(sdmin * (1 - scale) + sdmax * scale);
  GetHAL()->speed_set(sd, dphi);
}


void Smart::
InitPose(double x, double y, double theta)
{
}


void Smart::
SetPose(double x, double y, double theta)
{
}


void Smart::
GetPose(double & x, double & y, double & theta)
{
  const Frame & pose(GetServer()->GetTruePose());
  x = pose.X();
  y = pose.Y();
  theta = pose.Theta();
}


void Smart::
SetGoal(double timestep, const sfl::Goal & goal)
{
  m_replan_request = true;
  *m_goal = goal;
}


shared_ptr<const Goal> Smart::
GetGoal()
{
  return m_goal;
}


bool Smart::
GoalReached()
{
  return m_goal->DistanceReached(GetServer()->GetTruePose());
}

