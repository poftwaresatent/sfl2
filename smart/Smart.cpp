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
#include "PathDrawing.hpp"
#include <smartsfl/SmartAlgo.hpp>
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
  SmartPlanProxy(const SmartAlgo * smart_algo): m_smart_algo(smart_algo) {}
	
  virtual const estar::Facade * GetFacade()
	{ return m_smart_algo->GetEstar(); }
	
  virtual const sfl::GridFrame * GetFrame()
	{ return m_smart_algo->GetGridFrame(); }
	
  const SmartAlgo * m_smart_algo;
};


class SmartTraversabilityProxy: public TraversabilityProxy {
public:
	SmartTraversabilityProxy(const SmartAlgo * smart_algo)
		: m_smart_algo(smart_algo) {}
	
	virtual const sfl::TraversabilityMap * Get()
	{ return m_smart_algo->GetTraversability(); }
	
	const SmartAlgo * m_smart_algo;
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


class SmartGoalDrawing: public GoalInstanceDrawing {
public:
	SmartGoalDrawing(const std::string name, const SmartAlgo * smart_algo)
		: GoalInstanceDrawing(name, Goal()), m_smart_algo(smart_algo) {}
	
	virtual void Draw() {
		shared_ptr<const Goal> goal(m_smart_algo->GetGoal());
		if(goal)
			GoalInstanceDrawing::Draw(*goal);
	}
	
  const SmartAlgo * m_smart_algo;
};


Smart::
Smart(shared_ptr<RobotDescriptor> descriptor, const World & world)
  : RobotClient(descriptor, world, true),
    m_cheat(new CheatSheet(&world, GetServer())),
		m_smart_cs(new SmartColorScheme())
{
  expoparams params(descriptor);
  m_nscans = params.front_nscans;
  m_sick_channel = params.front_channel;
	
  double wheelbase(params.model_wheelbase);
  double wheelradius(params.model_wheelradius);
  double axlewidth(params.model_axlewidth);
	
	double carrot_distance;
	if( ! string_to(descriptor->GetOption("carrot_distance"), carrot_distance))
		carrot_distance = 5;
	
	double carrot_stepsize;
	if( ! string_to(descriptor->GetOption("carrot_stepsize"), carrot_stepsize))
		carrot_stepsize = 0.5;
	
	size_t carrot_maxnsteps;
	if( ! string_to(descriptor->GetOption("carrot_maxnsteps"), carrot_maxnsteps))
		carrot_maxnsteps = 30;
	
	double replan_distance;
	if( ! string_to(descriptor->GetOption("replan_distance"), replan_distance))
		replan_distance = 3;
	
	m_cheat_travmap = m_cheat->GetTravmap();
  if( ! m_cheat_travmap){
    cerr << "ERROR: Smart needs an a-priori traversability map.\n";
    exit(EXIT_FAILURE);
  }
	if( ! m_cheat_travmap->data){
    cerr << "ERROR: The a-priori traversability map is empty (data is NULL).\n";
    exit(EXIT_FAILURE);
  }
	
	shared_ptr<const TraversabilityMap> prior_travmap;
	shared_ptr<Mapper2d> mapper;
	const string use_travmap(descriptor->GetOption("use_travmap"));
	if(use_travmap == "cheat_discover")
		m_discover_travmap = true;
	else if(use_travmap == "cheat_apriori"){
		m_discover_travmap = false;
		prior_travmap = m_cheat_travmap;
	}
	else if(use_travmap == "mapper"){
		double robot_radius;
		if( ! string_to(descriptor->GetOption("robot_radius"), robot_radius)){
			cerr << "ERROR: Smart 'Option use_travmap mapper' also needs 'Option robot_radius'.\n";
			exit(EXIT_FAILURE);
		}
		mapper.reset(new Mapper2d(m_cheat_travmap->gframe,
															m_cheat_travmap->data->xsize,
															m_cheat_travmap->data->ysize,
															robot_radius,
															m_cheat_travmap->freespace,
															m_cheat_travmap->obstacle,
															m_cheat_travmap->name));
	}
	else{
    cerr << "ERROR: Option 'use_travmap' must be one of the following:\n"
				 << "  cheat_discover, cheat_apriori, or mapper\n";
    exit(EXIT_FAILURE);
	}
	
  bool single_step_estar(false);
  bool finish_estar(false);
	const string estar_mode(descriptor->GetOption("estar_mode"));
	if(estar_mode == "step")
		single_step_estar = true;		
	else if(estar_mode == "finish")
		finish_estar = true;
	
	string controller_name(descriptor->GetOption("controller_name"));
	if(controller_name == "")
		controller_name = "simple";
	shared_ptr<AckermannController>
		controller(CreateAckermannController(controller_name,
																				 AckermannModel(params.model_sd_max,
																												params.model_sdd_max,
																												params.model_phi_max,
																												params.model_phid_max,
																												wheelbase),
																				 &cerr));
  if( ! controller){
		cerr << "ERROR asl::CreateAckermannController() failed\n"
				 << "  controller_name: \"" << controller_name << "\"\n";
		exit(EXIT_FAILURE);
	}
	
	ostringstream err_os;
	m_smart_algo.reset(SmartAlgo::Create(replan_distance,
																			 carrot_distance,
																			 carrot_stepsize,
																			 carrot_maxnsteps,
																			 wheelbase,
																			 wheelradius,
																			 axlewidth,
																			 single_step_estar,
																			 finish_estar,
																			 mapper,
																			 controller,
																			 prior_travmap,
																			 &err_os));
	if( ! m_smart_algo){
		cerr << "ERROR asl::SmartAlgo::Create() failed\n " << err_os.str() << "\n";
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
	
	DefineBicycleDrive(wheelbase, wheelradius, axlewidth);

  AddLine(Line(-wheelradius, -axlewidth/2 -wheelradius,
							 -wheelradius,  axlewidth/2 +wheelradius));
  AddLine(Line(wheelbase +wheelradius, -axlewidth/2 -wheelradius,
							 wheelbase +wheelradius,  axlewidth/2 +wheelradius));
  AddLine(Line(-wheelradius, -axlewidth/2 -wheelradius,
							 wheelbase +wheelradius, -axlewidth/2 -wheelradius));
  AddLine(Line(-wheelradius,  axlewidth/2 +wheelradius,
							 wheelbase +wheelradius,  axlewidth/2 +wheelradius));
  
  const string & name(descriptor->name);
	AddDrawing(new SmartGoalDrawing(name + "_goaldrawing", m_smart_algo.get()));
  
	{
		shared_ptr<SmartPlanProxy> proxy(new SmartPlanProxy(m_smart_algo.get()));
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
		shared_ptr<TraversabilityProxy>
			smart_proxy(new SmartTraversabilityProxy(m_smart_algo.get()));
		AddDrawing(new TraversabilityDrawing(name + "_smart_travmap",
																				 smart_proxy));
		
		shared_ptr<TraversabilityProxy>
			cheat_proxy(new DirectTraversabilityProxy(m_cheat_travmap.get()));
		AddDrawing(new TraversabilityDrawing(name + "_cheat_travmap",
																				 cheat_proxy));
	}
	
  AddDrawing(new PathDrawing(name + "_carrot", this, 1));
}


//XXX rfct snippets void Smart::
// UpdatePlan(const Frame & pose, const GridFrame & gframe, const Scan & scan,
// 					 bool replan)
// {
// 	else if(m_discover_travmap){
// 		// fake it using the cheat travmap
// 		shared_ptr<Scan> myscan(m_sick->GetScanCopy());
// 		const Scan::array_t mydata(myscan->data);
// 		const size_t xsize(m_travmap->data->xsize);
// 		const size_t ysize(m_travmap->data->ysize);
// 		for(size_t i(0); i< mydata.size(); i++)
// 			gframe.DrawGlobalLine(pose.X(), pose.Y(),
// 														mydata[i].globx, mydata[i].globy,
// 														xsize, ysize, *m_cb);
// 		const size_t flushsize(3);
// 		if(replan || (m_cb->buf.size() > flushsize)){
// 			m_cb->flush();
// 			flushed = true;
// 		}
// 	}


// 		const double lpkey(m_estar->GetAlgorithm().GetLastPoppedKey());
// 		m_smart_cs->queue_bottom = lpkey;
// 	}
// }


void Smart::
PrepareAction(double timestep)
{
  m_mscan->UpdateAll();
	shared_ptr<const Scan> scan(m_mscan->CollectScans());
	
  double vtrans_cur, steer_cur;
  GetHAL()->speed_get( & vtrans_cur, & steer_cur);
	
	double vtrans_want, steer_want;
	ostringstream err_os;
	const SmartPlanThread::status_t
		status(m_smart_algo->ComputeAction(timestep,
																			 GetServer()->GetTruePose(),
																			 scan,
																			 vtrans_cur, steer_cur,
																			 vtrans_want, steer_want,
																			 & err_os));
	if(SmartPlanThread::ERROR == status){
		cerr << "asl::SmartAlgo::ComputeAction() failed:\n  "
				 << err_os.str() << "\n";
		exit(EXIT_FAILURE);
	}
	
	GetHAL()->speed_set(vtrans_want, steer_want);
	
	m_smart_cs->queue_bottom
		= m_smart_algo->GetEstar()->GetAlgorithm().GetLastPoppedKey();
	////	cerr << m_smart_cs->queue_bottom << "\n";
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
	m_smart_algo->SetGoal(goal);
}


shared_ptr<const Goal> Smart::
GetGoal()
{
  return m_smart_algo->GetGoal();
}


bool Smart::
GoalReached()
{
  return m_smart_algo->GoalReached();
}


const asl::path_t * Smart::
GetPath() const
{
	return m_smart_algo->GetPath();
}
