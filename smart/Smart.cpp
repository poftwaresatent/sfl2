/* -*- mode: C++; tab-width: 2 -*- */
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
#include "SmartNavFuncQuery.hpp"
#include <asl/path_tracking.hpp>
#include <sfl/api/Goal.hpp>
#include <sfl/api/Scanner.hpp>
#include <sfl/util/numeric.hpp>
#include <sfl/gplan/TraversabilityMap.hpp>
#include <sfl/gplan/GridFrame.hpp>
#include <sfl/gplan/Mapper2d.hpp>
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
#include <npm/common/TraversabilityDrawing.hpp>
#include <npm/common/CheatSheet.hpp>
#include <npm/estar/EstarDrawing.hpp>
#include <npm/estar/CarrotDrawing.hpp>
#include <iostream>
#include <math.h>


#ifdef NPM_DEBUG
# define PDEBUG PDEBUG_OUT
#else // ! NPM_DEBUG
# define PDEBUG PDEBUG_OFF
#endif // NPM_DEBUG
#define PVDEBUG PDEBUG_OFF


using namespace npm;
using namespace sfl;
using namespace asl;
using namespace estar;
using namespace boost;
using namespace std;


class SmartPlanProxy: public PlanProxy {
public:
  SmartPlanProxy(Smart * smart): m_smart(smart) {}
	
  virtual const estar::Facade * GetFacade()
	{ return m_smart->m_estar.get(); }
	
  virtual const sfl::GridFrame * GetFrame()
	{ return & m_smart->m_travmap->gframe; }
	
  Smart * m_smart;
};


class SmartCarrotProxy: public CarrotProxy {
public:
  SmartCarrotProxy(const Smart * _smart): smart(_smart) {}
  
  virtual const estar::carrot_trace * GetCarrotTrace() const
  { return smart->m_carrot_trace.get(); }
  
  virtual const sfl::GridFrame * GetGridFrame() const
  { return & smart->m_travmap->gframe; }
  
  const Smart * smart;
};


class MapperTraversabilityProxy: public TraversabilityProxy {
public:
	MapperTraversabilityProxy(Mapper2d * mapper)
		: m_mapper(mapper) {}
	
	virtual const sfl::TraversabilityMap * Get()
	{ return m_mapper->getTravMap().get(); }
	
	Mapper2d * m_mapper;
};


Smart::
Smart(shared_ptr<RobotDescriptor> descriptor, const World & world)
  : RobotClient(descriptor, world, true),
    single_step_estar(false),
    m_goal(new Goal()),
    m_cheat(new CheatSheet(&world, GetServer())),
    m_carrot_proxy(new SmartCarrotProxy(this)),
    m_replan_request(false),
		m_plan_status(PlanThread::PLANNING)
{
	if(descriptor->GetOption("use_travmap") == "cheat")
		m_travmap = m_cheat->GetTravmap();
	else{
		m_mapper.reset(new Mapper2d());
		m_travmap = m_mapper->getTravMap();
	}
  if( ! m_travmap){
    cerr << "ERROR: Smart needs a traversability map.\n";
    exit(EXIT_FAILURE);
  }
	
	if(descriptor->GetOption("estar_mode") == "step")
		single_step_estar = true;		
	
  expoparams params(descriptor);
  m_nscans = params.front_nscans;
  m_sick_channel = params.front_channel;
  m_wheelbase = params.model_wheelbase;
  m_wheelradius = params.model_wheelradius;
  m_axlewidth = params.model_axlewidth;

  m_controller.
    reset(new AckermannController(AckermannModel(params.model_sd_max,
						 params.model_sdd_max,
						 params.model_phi_max,
						 params.model_phid_max,
						 m_wheelbase)));
  
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
  
  const string & name(descriptor->name);
  AddDrawing(new GoalInstanceDrawing(name + "_goaldrawing",
				     *m_goal));
  
	{
		shared_ptr<SmartPlanProxy> proxy(new SmartPlanProxy(this));
		AddDrawing(new EstarDrawing(name + "_estar_meta",
																proxy, EstarDrawing::META));
		AddDrawing(new EstarDrawing(name + "_estar_value",
																proxy, EstarDrawing::VALUE));
		AddDrawing(new EstarDrawing(name + "_estar_queue",
																proxy, EstarDrawing::QUEUE));
		AddDrawing(new EstarDrawing(name + "_estar_upwind",
																proxy, EstarDrawing::UPWIND));
	}
	
	{
		shared_ptr<TraversabilityProxy> proxy;
		if(m_mapper)
			proxy.reset(new MapperTraversabilityProxy(m_mapper.get()));
		else
			proxy.reset(new DirectTraversabilityProxy(m_cheat->GetTravmap().get()));
		AddDrawing(new TraversabilityDrawing(name + "_travcs", proxy));
		if(m_mapper)
			proxy.reset(new DirectTraversabilityProxy(m_cheat->GetTravmap().get()));
		AddDrawing(new TraversabilityDrawing(name + "_cheat_trav", proxy));
	}
	
  AddDrawing(new CarrotDrawing(name + "_carrot", m_carrot_proxy, 1));
}


void Smart::
HandleReplanRequest()
{
  if( ! m_replan_request)
		return;
	m_replan_request = false;
	
	const GridFrame & gframe(m_travmap->gframe);
	shared_ptr<const array2d<int> > travdata(m_travmap->data);
	if( ! travdata){
		cerr << "ERROR: Invalid traversability map (no data).\n";
		exit(EXIT_FAILURE);
	}
	
	if( ! m_estar){
		m_estar.reset(Facade::CreateDefault(travdata->xsize, travdata->ysize,
																				gframe.Delta()));
		
		const double scale(1.0 / (m_travmap->obstacle - m_travmap->freespace));
		for(size_t ix(0); ix < travdata->xsize; ++ix)
			for(size_t iy(0); iy < travdata->ysize; ++iy){
				const double meta(scale * (m_travmap->obstacle - (*travdata)[ix][iy]));
				m_estar->InitMeta(ix, iy, sfl::boundval(0.0, meta, 1.0));
			}
	}
	else
		m_estar->RemoveAllGoals();
	
	double goalx(m_goal->X());
	double goaly(m_goal->Y());
	gframe.From(goalx, goaly);
	m_goalregion.reset(new Region(m_goal->Dr(), gframe.Delta(),
																goalx, goaly, travdata->xsize, travdata->ysize));
	if(m_goalregion->GetArea().empty()){
		PDEBUG_ERR("ERROR: empty goal area()!\n");
		exit(EXIT_FAILURE);
	}
	
	////    m_estar->AddGoal(*m_goalregion);
	for(Region::indexlist_t::const_iterator
				in(m_goalregion->GetArea().begin());
			in != m_goalregion->GetArea().end(); ++in)
		m_estar->AddGoal(in->x, in->y, in->r);
	
	if( ! m_plan_thread)
		m_plan_thread.reset(new PlanThread(m_estar, gframe,
																			 travdata->xsize, travdata->ysize));
}


/**
	 \todo provide a way to just set those E-Star meta values that have
	 actually changed
*/
bool Smart::
UpdatePlan()
{
  // Updating traversability Map
	const Frame mypose(GetServer()->GetTruePose());
	if(m_mapper)
		m_mapper->update(mypose , *m_sick);
	
	////m_plan_thread->Stop();
	
	shared_ptr<const array2d<int> > travdata(m_travmap->data);
	if( ! travdata){
		cerr << "ERROR: Invalid traversability map (no data).\n";
		return false;
	}
	
	const double obst(m_estar->GetObstacleMeta());
	for(size_t ii(0); ii < travdata->xsize; ++ii)
		for(size_t jj(0); jj < travdata->ysize; ++jj)
			if((*travdata)[ii][jj] >= m_travmap->obstacle)
				m_estar->SetMeta(ii, jj, obst);
	
	////m_plan_thread->Start(100000);

	m_plan_status = m_plan_thread->GetStatus(mypose.X(), mypose.Y());
	if(PlanThread::HAVE_PLAN == m_plan_status)
		return true;
	
  if(single_step_estar){
    m_plan_thread->Step();
		m_plan_status = m_plan_thread->GetStatus(mypose.X(), mypose.Y());
	}
  else
    while(m_plan_status == PlanThread::PLANNING){
      m_plan_thread->Step();
			m_plan_status = m_plan_thread->GetStatus(mypose.X(), mypose.Y());
		}
	
	return true;
}


bool Smart::
ComputePath(const Frame & pose, const GridFrame & gframe, path_t & path)
{
  if( ! m_carrot_trace)
    m_carrot_trace.reset(new carrot_trace());
  else
    m_carrot_trace->clear();
  const double carrot_distance(5); // XXX to do: magic numbers...
  const double carrot_stepsize(0.5);
  const size_t carrot_maxnsteps(30);
  double robx_grid(pose.X());
  double roby_grid(pose.Y());
  gframe.From(robx_grid, roby_grid);
  const int result(trace_carrot(*m_estar, robx_grid, roby_grid,
																carrot_distance, carrot_stepsize,
																carrot_maxnsteps, *m_carrot_trace));
  if(0 > result){
    PVDEBUG("FAILED compute_carrot()\n");
    return false;
  }
  else{
    if(1 == result)
      PVDEBUG("WARNING: carrot didn't reach distance %g\n", carrot_distance);
    PVDEBUG("carrot.value = %g\n", m_carrot_trace->back().value);
    if(m_carrot_trace->back().value <= 3 * carrot_stepsize){ // XXX magic numbers!
      PVDEBUG("carrot on goal border, appending goal point to carrot");
      double foox(m_goal->X());
      double fooy(m_goal->Y());
      gframe.From(foox, fooy);
      m_carrot_trace->push_back(carrot_item(foox, fooy, 0, 0, 0, true));
    }
  }
  
  for(size_t ii(0); ii < m_carrot_trace->size(); ++ii){
    carrot_item item((*m_carrot_trace)[ii]);
    gframe.To(item.cx, item.cy);
    gframe.RotateTo(item.gradx, item.grady);
    path.push_back(path_element(path_point(item.cx, item.cy),
				path_point(item.gradx, item.grady),
				item.value, item.degenerate));
  }
	
	return true;
}


void Smart::
PrepareAction(double timestep)
{
  m_sick->Update();
	
	HandleReplanRequest();
  if( ! m_plan_thread){
    PDEBUG_ERR("BUG: no plan thread late in Smart::PrepareAction()!\n");
		exit(EXIT_FAILURE);
  }

  double v_trans, steer;
  GetHAL()->speed_get(&v_trans, &steer);
	
	UpdatePlan();
  switch(m_plan_status){
  case PlanThread::HAVE_PLAN:
		// do the stuff after this switch
    break;
  case PlanThread::PLANNING:
    PDEBUG("wave hasn't crossed robot yet\n");
    GetHAL()->speed_set(0, steer);
    return;
  case PlanThread::UNREACHABLE:
    PDEBUG_ERR("ERROR: no path to goal\n");
    exit(EXIT_FAILURE);
  case PlanThread::OUT_OF_GRID:
    PDEBUG_ERR("ERROR: robot is outside grid\n");
    exit(EXIT_FAILURE);
  case PlanThread::IN_OBSTACLE:
    PDEBUG_ERR("ERROR: robot is inside an obstacle\n");
    exit(EXIT_FAILURE);
  default:
    PDEBUG_ERR("BUG: unhandled result %d of m_plan_thread->GetStatus()\n",
							 m_plan_status);
    exit(EXIT_FAILURE);
  }
	
  path_t path;
	if( ! ComputePath(GetServer()->GetTruePose(), m_travmap->gframe, path)){
    GetHAL()->speed_set(0, steer);
		return;
	}
  
	// beware: (v_trans, steer) have to be the *current* motion state
  const int result(m_controller->ComputeCommand(GetServer()->GetTruePose(),
																								path,
																								SmartNavFuncQuery(this),
																								v_trans, steer, timestep));
  if(0 != result){
    PVDEBUG("FAILED ComputeCommand()\n");
    GetHAL()->speed_set(0, steer);
    return;
  }
  if( ! m_controller->GetCommand(v_trans, steer)){
    PVDEBUG("FAILED GetCommand()\n");
    GetHAL()->speed_set(0, steer);
    return;
  }
  GetHAL()->speed_set(v_trans, steer);
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

