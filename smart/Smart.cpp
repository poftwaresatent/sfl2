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
#include <sfl/api/Scan.hpp>
#include <sfl/api/Scanner.hpp>
#include <sfl/api/Multiscanner.hpp>
#include <sfl/api/Odometry.hpp>
#include <sfl/util/numeric.hpp>
#include <sfl/gplan/TraversabilityMap.hpp>
#include <sfl/gplan/GridFrame.hpp>
#include <sfl/gplan/Mapper2d.hpp>
#include <estar/Facade.hpp>
#include <estar/Algorithm.hpp>	// for delete in shared_ptr through Facade
#include <estar/Kernel.hpp>	// for delete in shared_ptr through Facade
#include <estar/Region.hpp>
#include <estar/Grid.hpp>
#include <estar/graphics.hpp>
#include <estar/numeric.hpp>
#include <npm/robox/expoparams.hpp>
#include <npm/common/Lidar.hpp>
#include <npm/common/HAL.hpp>
#include <npm/common/RobotServer.hpp>
#include <npm/common/GoalInstanceDrawing.hpp>
#include <npm/common/TraversabilityDrawing.hpp>
#include <npm/common/CheatSheet.hpp>
#include <npm/common/util.hpp>
#include <npm/common/wrap_gl.hpp>
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


class SmartColorScheme: public gfx::ColorScheme {
public:
	SmartColorScheme(): smart_value(0), queue_bottom(0) {}
	virtual void Set(double value) const {
		if(value >= estar::infinity)
			glColor3d(0, 0, 0.4);
		else{
			double green;
			if(value <= smart_value)
				green = sfl::minval(1.0, value / smart_value) * 0.4;
			else
				green = 0.5;
			double red;
			if(value <= queue_bottom){
				if(smart_value > queue_bottom)
					red = sfl::minval(1.0, value / queue_bottom) * 0.4;
				else
					red = 0;
			}
			else{
				red = 0.5;
				green = 0;
			}
			glColor3d(red, green, sfl::minval(1.0,
																				value / sfl::maxval(estar::epsilon,
																														smart_value)));
		}
	}
	double smart_value;
	double queue_bottom;
};


class SmartDrawCallback: public GridFrame::draw_callback {
public:
	class index_t: public array2d<size_t> {
	public:
		index_t(size_t _ix, size_t _iy): ix(_ix), iy(_iy) {}
		bool operator < (const index_t rhs) const
		{ return (ix < rhs.ix) || ((ix == rhs.ix) && (iy < rhs.iy)); }
		const size_t ix;
		const size_t iy;
	};
	
	typedef map<index_t, double> buf_t;
	typedef set<index_t> known_t;
	typedef buf_t::const_iterator it_t;
	
	SmartDrawCallback(const sfl::TraversabilityMap * _travmap,
										Facade * _estar)
		: travmap(_travmap), estar(_estar),
			scale(1.0 / (_travmap->obstacle - _travmap->freespace)),
			obstacle(_travmap->obstacle) {}
	
	double compute(int trav) const
	{ return sfl::boundval(0.0, scale * (obstacle - trav), 1.0); }
	
	void flush() {
		for(it_t it(buf.begin()); it != buf.end(); ++it)
			estar->SetMeta(it->first.ix, it->first.iy, it->second);
		buf.clear();
	}
	
	virtual void operator () (size_t ix, size_t iy) {
		const array2d<int> * data(travmap->data.get());
		if(( ! data) || (ix >= data->xsize) || (iy >= data->ysize))
			return;
		const index_t idx(ix, iy);
		if(known.find(idx) != known.end())
			return;
		const double meta(compute((*data)[ix][iy]));
		buf.insert(make_pair(idx, meta));
		known.insert(idx);
	}
	
	const sfl::TraversabilityMap * travmap;
	mutable Facade * estar;
	const double scale;
	const double obstacle;
	buf_t buf;
	known_t known;
};


Smart::
Smart(shared_ptr<RobotDescriptor> descriptor, const World & world)
  : RobotClient(descriptor, world, true),
    single_step_estar(false),
		finish_estar(false),
    m_goal(new Goal()),
    m_cheat(new CheatSheet(&world, GetServer())),
    m_carrot_proxy(new SmartCarrotProxy(this)),
		m_smart_cs(new SmartColorScheme()),
    m_replan_request(false),
		m_plan_status(PlanThread::PLANNING)
{
	shared_ptr<const TraversabilityMap> cheat_travmap(m_cheat->GetTravmap());
  if( ! cheat_travmap){
    cerr << "ERROR: Smart needs an a-priori traversability map.\n";
    exit(EXIT_FAILURE);
  }
	if( ! cheat_travmap->data){
    cerr << "ERROR: The a-priori traversability map is empty (data is NULL).\n";
    exit(EXIT_FAILURE);
  }
	
	const string use_travmap(descriptor->GetOption("use_travmap"));
	if(use_travmap == "cheat_discover"){
		m_travmap = cheat_travmap;
		m_discover_travmap = true;
	}
	else if(use_travmap == "cheat_apriori"){
		m_travmap = cheat_travmap;
		m_discover_travmap = false;
	}
	else if(use_travmap == "mapper"){
		double robot_radius;
		if( ! string_to(descriptor->GetOption("robot_radius"), robot_radius)){
			cerr << "ERROR: Smart 'Option use_travmap mapper' also needs 'Option robot_radius'.\n";
			exit(EXIT_FAILURE);
		}
		m_mapper.reset(new Mapper2d(cheat_travmap->gframe,
																cheat_travmap->data->xsize,
																cheat_travmap->data->ysize,
																robot_radius,
																cheat_travmap->freespace,
																cheat_travmap->obstacle,
																cheat_travmap->name));
		m_travmap = m_mapper->getTravMap();
	}
	else{
    cerr << "ERROR: Option 'use_travmap' must be one of the following:\n"
				 << "  cheat_discover, cheat_apriori, or mapper\n";
    exit(EXIT_FAILURE);
	}
	
	
	const string estar_mode(descriptor->GetOption("estar_mode"));
	if(estar_mode == "step")
		single_step_estar = true;		
	else if(estar_mode == "finish")
		finish_estar = true;
	
	if( ! string_to(descriptor->GetOption("replan_distance"), m_replan_distance))
		m_replan_distance = 3;
	
  expoparams params(descriptor);
  m_nscans = params.front_nscans;
  m_sick_channel = params.front_channel;
  m_wheelbase = params.model_wheelbase;
  m_wheelradius = params.model_wheelradius;
  m_axlewidth = params.model_axlewidth;

	string controller_name(descriptor->GetOption("controller_name"));
	if(controller_name == "")
		controller_name = "simple";
  m_controller.
    reset(asl::CreateAckermannController(controller_name,
																				 AckermannModel(params.model_sd_max,
																												params.model_sdd_max,
																												params.model_phi_max,
																												params.model_phid_max,
																												m_wheelbase),
																				 &cerr));
  if( ! m_controller){
		cerr << "something went wrong in asl::CreateAckermannController()\n";
		exit(EXIT_FAILURE);
	}
	
	shared_ptr<Odometry> odo(new Odometry(GetHAL(), RWlock::Create("smart")));	
	m_mscan.reset(new Multiscanner(odo));
	m_sick = DefineLidar(Frame(params.front_mount_x,
														 params.front_mount_y,
														 params.front_mount_theta),
											 params.front_nscans,
											 params.front_rhomax,
											 params.front_phi0,
											 params.front_phirange,
											 params.front_channel)->GetScanner();
	m_mscan->Add(m_sick);
	
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
																proxy, EstarDrawing::VALUE, m_smart_cs.get()));
		AddDrawing(new EstarDrawing(name + "_estar_queue",
																proxy, EstarDrawing::QUEUE));
		AddDrawing(new EstarDrawing(name + "_estar_upwind",
																proxy, EstarDrawing::UPWIND));
		AddDrawing(new EstarDrawing(name + "_estar_obst",
																proxy, EstarDrawing::OBST));
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


bool Smart::
HandleReplanRequest(const GridFrame & gframe)
{
  if( ! m_replan_request)
		return false;
	m_replan_request = false;
	
	shared_ptr<const array2d<int> > travdata(m_travmap->data);
	if( ! travdata){
		cerr << "ERROR: Invalid traversability map (no data).\n";
		exit(EXIT_FAILURE);
	}
	
	if( ! m_estar)
		m_estar.reset(Facade::CreateDefault(travdata->xsize, travdata->ysize,
																				gframe.Delta()));
	else{
		m_estar->RemoveAllGoals();
	}
	
	double goalx(m_goal->X());
	double goaly(m_goal->Y());
	gframe.From(goalx, goaly);
	m_goalregion.reset(new Region(m_goal->Dr(), gframe.Delta(), goalx, goaly,
																travdata->xsize, travdata->ysize));
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

	return true;
}


/**
	 \todo provide a way to just set those E-Star meta values that have
	 actually changed
*/
void Smart::
UpdatePlan(const Frame & pose, const GridFrame & gframe, const Scan & scan,
					 bool replan)
{
	bool flushed(false);
	if( ! m_cb){
		m_cb.reset(new SmartDrawCallback(m_travmap.get(), m_estar.get()));
		if( ! m_discover_travmap){
			for(size_t ix(0); ix < m_travmap->data->xsize; ++ix)
				for(size_t iy(0); iy < m_travmap->data->ysize; ++iy)
					(*m_cb)(ix, iy);
			m_cb->flush();
			flushed = true;
		}
	}
	
  // Updating traversability Map
	if(m_mapper){
		// use our sfl::Mapper2d instance
		m_mapper->update(pose , scan, m_cb.get());
		if(m_last_plan_pose){
	 		const double dist(sqrt(sqr(m_last_plan_pose->X() - pose.X())
	 													 + sqr(m_last_plan_pose->Y() - pose.Y())));
	 		if((dist > m_replan_distance) || replan){
				*m_last_plan_pose = pose;
				m_cb->flush();
				flushed = true;
			}
	 	} // ! m_last_plan_pose
	 	else{
	 		m_last_plan_pose.reset(new Frame(pose));
			m_cb->flush();
			flushed = true;
	 	}
	} // ! m_mapper
	else if(m_discover_travmap){
		// fake it using the cheat travmap
		shared_ptr<Scan> myscan(m_sick->GetScanCopy());
		const Scan::array_t mydata(myscan->data);
		const size_t xsize(m_travmap->data->xsize);
		const size_t ysize(m_travmap->data->ysize);
		for(size_t i(0); i< mydata.size(); i++)
			gframe.DrawGlobalLine(pose.X(), pose.Y(),
														mydata[i].globx, mydata[i].globy,
														xsize, ysize, *m_cb);
		const size_t flushsize(3);
		if(replan || (m_cb->buf.size() > flushsize)){
			m_cb->flush();
			flushed = true;
		}
	}
	else
		flushed = true;
	
	// do the actual planning, if there's anything to do
	if(flushed || (PlanThread::HAVE_PLAN != m_plan_status)){
		if(single_step_estar){
			m_plan_thread->Step();
			m_plan_status = m_plan_thread->GetStatus(pose.X(), pose.Y());
		}
		else{
			if(finish_estar){
				while(m_estar->HaveWork())
					m_plan_thread->Step();
				m_plan_status = m_plan_thread->GetStatus(pose.X(), pose.Y());
			}
			else
				while(m_plan_status == PlanThread::PLANNING){
					m_plan_thread->Step();
					m_plan_status = m_plan_thread->GetStatus(pose.X(), pose.Y());
				}
		}
		const double lpkey(m_estar->GetAlgorithm().GetLastPoppedKey());
		m_smart_cs->queue_bottom = lpkey;
	}
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
  m_mscan->UpdateAll();
	shared_ptr<const Scan> scan(m_mscan->CollectScans());
	
	const GridFrame & gframe(m_travmap->gframe);
	const bool replan(HandleReplanRequest(gframe));
  if( ! m_plan_thread){
    PDEBUG_ERR("BUG: no plan thread late in Smart::PrepareAction()!\n");
		exit(EXIT_FAILURE);
  }

  double v_trans, steer;
  GetHAL()->speed_get(&v_trans, &steer);
	
	const Frame & pose(GetServer()->GetTruePose());
	UpdatePlan(pose, gframe, *scan, replan);
  switch(m_plan_status){
  case PlanThread::HAVE_PLAN:
		// do the stuff after this switch
    break;
  case PlanThread::PLANNING:
    PDEBUG("wave hasn't crossed robot yet\n");
    GetHAL()->speed_set(0, steer);
    return;
  case PlanThread::AT_GOAL:
    PDEBUG("at goal\n");
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

  const GridFrame::index_t idx(gframe.GlobalIndex(pose.X(), pose.Y()));
	shared_ptr<const array2d<int> > travdata(m_travmap->data);
	if(travdata){
		if((idx.v0 < travdata->xsize) && (idx.v1 < travdata->ysize))
			m_smart_cs->smart_value = m_estar->GetValue(idx.v0, idx.v1);
	}
	
  path_t path;
	if( ! ComputePath(GetServer()->GetTruePose(), gframe, path)){
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
