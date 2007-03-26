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
#include <sfl/api/Multiscanner.hpp>
#include <sfl/api/Odometry.hpp>
#include <sfl/util/numeric.hpp>
#include <sfl/gplan/TraversabilityMap.hpp>
#include <sfl/gplan/Mapper2d.hpp>
#include <estar/Facade.hpp>
#include <estar/Algorithm.hpp>
#include <estar/graphics.hpp>
#include <estar/numeric.hpp>
#include <npm/robox/expoparams.hpp>
#include <npm/common/Lidar.hpp>
#include <npm/common/HAL.hpp>
#include <npm/common/RobotServer.hpp>
#include <npm/common/RobotDescriptor.hpp>
#include <npm/common/GoalInstanceDrawing.hpp>
#include <npm/common/TraversabilityDrawing.hpp>
#include <npm/common/CheatSheet.hpp>
#include <npm/common/World.hpp>
#include <npm/common/util.hpp>
#include <npm/common/wrap_gl.hpp>
#include <npm/estar/EstarDrawing.hpp>
#include <iostream>


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


class SmartPlanProxy:
	public PlanProxy,
	public KeyListener
{
public:
  SmartPlanProxy(const SmartAlgo * smart_algo, bool enabled)
		: m_smart_algo(smart_algo), m_enabled(enabled) {}
	
  virtual const estar::Facade * GetFacade()
	{ return m_smart_algo->GetEstar(); }
	
  virtual const sfl::GridFrame * GetFrame()
	{ return & m_smart_algo->GetGridFrame(); }
	
	virtual bool Enabled() const { return m_enabled; }
	
	virtual void KeyPressed(unsigned char key)
	{ if('o' == key) m_enabled = ! m_enabled; }
	
  const SmartAlgo * m_smart_algo;
	bool m_enabled;
};


class SmartTraversabilityProxy:
	public TraversabilityProxy,
	public KeyListener
{
public:
	SmartTraversabilityProxy(const SmartAlgo * smart_algo, bool enabled)
		: m_smart_algo(smart_algo), m_enabled(enabled) {}
	
	virtual const sfl::TraversabilityMap * Get()
	{ return m_smart_algo->GetTraversability(); }
	
	virtual bool Enabled() const { return m_enabled; }
	
	virtual void KeyPressed(unsigned char key)
	{ if('o' == key) m_enabled = ! m_enabled; }
	
	const SmartAlgo * m_smart_algo;
	bool m_enabled;
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


class MetaColorScheme: public gfx::ColorScheme {
public:
	virtual void Set(double value) const {
		if(value >= (1 - estar::epsilon))
			glColor3d(0, 0, 1);
		else if(value <= estar::epsilon)
			glColor3d(1, 0, 0);
		else
			glColor3d(value, value, value);
	}
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
		
	if(descriptor->GetOption("use_travmap") != ""){
    cerr << "ERROR: Smart option 'use_travmap' is deprecated.\n"
				 << "  Use the enable_mapper option (defaults to 'true')!\n";
    exit(EXIT_FAILURE);
	}
	
	m_enable_mapper = true;
	string_to(descriptor->GetOption("enable_mapper"), m_enable_mapper);
	string traversability_file(descriptor->GetOption("traversability_file"));
	if(traversability_file == ""){
    cerr << "ERROR: Smart needs an a-priori traversability map.\n"
				 << "  Use the traversability_file option!\n";
    exit(EXIT_FAILURE);
  }
	double robot_radius(1.5);
	string_to(descriptor->GetOption("robot_radius"), robot_radius);
	double buffer_zone(robot_radius);
	string_to(descriptor->GetOption("buffer_zone"), buffer_zone);
	m_mapper = Mapper2d::Create(robot_radius, buffer_zone,
															traversability_file, &cerr);
	if( ! m_mapper){
    cerr << "ERROR: Could not create mapper, see error messages above.\n";
    exit(EXIT_FAILURE);
	}
	
	int estar_step(-1);
	string_to(descriptor->GetOption("estar_step"), estar_step);
	
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
	
	double wavefront_buffer(2);
	string_to(descriptor->GetOption("wavefront_buffer"), wavefront_buffer);
	
	string goalmgr_filename(descriptor->GetOption("goalmgr_filename"));
	if(goalmgr_filename != ""){
		if(descriptor->HaveGoals()){
			cerr << "ERROR goalmgr_filename option conflicts with Goal statements.\n"
					 << "  Cannot mix npm Goal with Option goalmgr_filename.\n";
			exit(EXIT_FAILURE);
		}
	}
	else if( ! descriptor->HaveGoals()){
		cerr << "ERROR no goals and no goalmgr_filename option given.\n";
		exit(EXIT_FAILURE);
	}
	
	ostringstream err_os;
	m_smart_algo.reset(SmartAlgo::Create(replan_distance,
																			 wavefront_buffer,
																			 carrot_distance,
																			 carrot_stepsize,
																			 carrot_maxnsteps,
																			 estar_step,
																			 m_mapper,
																			 controller,
																			 goalmgr_filename,
																			 &err_os));
	if( ! m_smart_algo){
		cerr << "ERROR asl::SmartAlgo::Create() failed\n " << err_os.str() << "\n";
		exit(EXIT_FAILURE);
	}
	m_goal_manager = m_smart_algo->GetGoalManager();
	
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
  
	bool slow_drawing_enabled(true);
	string_to(descriptor->GetOption("slow_drawing_enabled"),
						slow_drawing_enabled);
	
	{
		shared_ptr<SmartPlanProxy>
			slow_proxy(new SmartPlanProxy(m_smart_algo.get(), slow_drawing_enabled));
		world.AddKeyListener(slow_proxy);
		shared_ptr<SmartPlanProxy>
			fast_proxy(new SmartPlanProxy(m_smart_algo.get(), true));
		shared_ptr<MetaColorScheme> mcs(new MetaColorScheme());
		AddDrawing(new EstarDrawing(name + "_estar_meta",
																slow_proxy, EstarDrawing::META, mcs));
		AddDrawing(new EstarDrawing(name + "_estar_value",
																slow_proxy, EstarDrawing::VALUE, m_smart_cs));
		AddDrawing(new EstarDrawing(name + "_estar_queue",
																fast_proxy, EstarDrawing::QUEUE));
		AddDrawing(new EstarDrawing(name + "_estar_upwind",
																slow_proxy, EstarDrawing::UPWIND));
		AddDrawing(new EstarDrawing(name + "_estar_obst",
																slow_proxy, EstarDrawing::OBST));
		AddDrawing(new EstarDrawing(name + "_estar_status",
																slow_proxy, EstarDrawing::STATUS));
	}
	
	{
		shared_ptr<SmartTraversabilityProxy>
			smart_proxy(new SmartTraversabilityProxy(m_smart_algo.get(),
																							 slow_drawing_enabled));
		world.AddKeyListener(smart_proxy);
		AddDrawing(new TraversabilityDrawing(name + "_smart_travmap",
																				 smart_proxy));
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


void Smart::
PrepareAction(double timestep)
{
  m_mscan->UpdateAll();
	shared_ptr<const Scan> scan(m_mscan->CollectScans());
	
  double vtrans_cur, steer_cur;
  GetHAL()->speed_get( & vtrans_cur, & steer_cur);
	
	//RFCT// to do: handle m_enable_mapper here (currently in SmartAlgo)
	
	double vtrans_want, steer_want;
	ostringstream err_os;
	const SmartPlanThread::status_t
		status(m_smart_algo->ComputeAction(timestep,
																			 GetServer()->GetTruePose(),
																			 scan,
																			 vtrans_cur, steer_cur,
																			 vtrans_want, steer_want,
																			 & err_os));
	switch(status){
	case SmartPlanThread::HAVE_PLAN: break;
	case SmartPlanThread::PLANNING: break;
	case SmartPlanThread::AT_GOAL:
		if(m_goal_manager){
			if( ! m_smart_algo->SetNextGoal()){
				cerr << "\nERROR in Smart::PrepareAction():"
						 << " m_smart_algo->SetNextGoal() failed\n";
				exit(EXIT_FAILURE);
			}
		}
		break;
	case SmartPlanThread::UNREACHABLE:
		cerr << "\nERROR in Smart::PrepareAction(): goal is unreachable\n";
		exit(EXIT_FAILURE);
	case SmartPlanThread::OUT_OF_GRID:
		cerr << "\nERROR in Smart::PrepareAction(): robot is out of grid\n";
		exit(EXIT_FAILURE);
	case SmartPlanThread::IN_OBSTACLE:
		cerr << "\nERROR in Smart::PrepareAction(): robot is in obstacle\n";
		exit(EXIT_FAILURE);
	case SmartPlanThread::ERROR:
		cerr << "\nERROR in Smart::PrepareAction(): ComputeAction() says\""
				 << err_os.str() << "\"\n";
		exit(EXIT_FAILURE);
	default:
		cerr << "\nERROR in Smart::PrepareAction(): unhandled retval " << status
				 << " from ComputeAction()\n";
		exit(EXIT_FAILURE);
	}
	
	//XXX rfct // 		const GridFrame::index_t idx(gframe.GlobalIndex(pose.X(), pose.Y()));
	// // 		shared_ptr<const array2d<int> > travdata(m_travmap->data);
	// // 		if(travdata){
	// // 			if((idx.v0 < travdata->xsize) && (idx.v1 < travdata->ysize))
	// // 				m_smart_cs->smart_value = m_estar->GetValue(idx.v0, idx.v1);
	// // 		}
	
	GetHAL()->speed_set(vtrans_want, steer_want);
	
	m_smart_cs->queue_bottom
		= m_smart_algo->GetEstar()->GetAlgorithm().GetLastPoppedKey();
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


const asl::trajectory_t * Smart::
GetTrajectory() const
{
	return m_smart_algo->GetTrajectory();
}

bool Smart::
GetRefpoint(asl::path_point &ref_point) const
{
	return m_smart_algo->GetRefpoint(ref_point);
}

