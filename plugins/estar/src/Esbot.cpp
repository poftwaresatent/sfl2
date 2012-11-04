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


#include "Esbot.hpp"
#include "PNF.hpp"
#include "PNFDrawing.hpp"
#include "EstarDrawing.hpp"
#include "PNFCamera.hpp"
#include "CarrotDrawing.hpp"
#include "../robox/OCamera.hpp"
#include "../robox/ODrawing.hpp"
#include "../robox/DODrawing.hpp"
#include "../robox/DWDrawing.hpp"
#include "../robox/robox_parameters.hpp"
#include "../common/pdebug.hpp"
#include "../common/Lidar.hpp"
#include "../common/GoalInstanceDrawing.hpp"
#include "../common/OdometryDrawing.hpp"
#include "../common/StillCamera.hpp"
#include "../common/HAL.hpp"
#include "../common/CheatSheet.hpp"
#include "../common/DiffDrive.hpp"
#include "../common/Manager.hpp"
#include "../common/RobotDescriptor.hpp"
#include <sfl/util/strutil.hpp>
#include <sfl/util/Hull.hpp>
#include <sfl/util/numeric.hpp>
#include <sfl/api/RobotModel.hpp>
#include <sfl/api/Odometry.hpp>
#include <sfl/api/Scanner.hpp>
#include <sfl/api/Multiscanner.hpp>
#include <sfl/api/MotionController.hpp>
#include <sfl/dwa/DynamicWindow.hpp>
#include <sfl/dwa/DistanceObjective.hpp>
#include <sfl/dwa/HeadingObjective.hpp>
#include <sfl/dwa/SpeedObjective.hpp>
#include <sfl/gplan/GridFrame.hpp>
#include <estar/Facade.hpp>
#include <estar/dump.hpp>
#include <pnf/Flow.hpp>
#include <iostream>
#include <set>


using namespace npm;

using sfl::Frame;
using sfl::Hull;
using sfl::HullIterator;
using sfl::Line;
using sfl::Polygon;
using sfl::Pose;
using sfl::Goal;
using sfl::RobotModel;
using sfl::LegacyDynamicWindow;
using sfl::Odometry;
using sfl::Multiscanner;
using sfl::MotionController;
using sfl::GridFrame;
using sfl::Scan;
using sfl::sqr;
using sfl::absval;
using sfl::RWlock;
using sfl::string_to;

using namespace estar;
using namespace boost;
using namespace std;


static RobotModel::Parameters CreateRobotParameters(expo_parameters & params);
static shared_ptr<Hull> CreateHull();  


class EnvdistProxy: public PlanProxy {
public:
  EnvdistProxy(Esbot * esbot): m_esbot(esbot) {}
  
  virtual const estar::Facade * GetFacade() {
    if( ! m_esbot->GetPNF()) return 0;
    return &m_esbot->GetPNF()->GetFlow()->GetEnvdist();
  }
  
  virtual const sfl::GridFrame * GetFrame() {
    if( ! m_esbot->GetPNF()) return 0;
    return m_esbot->GetPNF()->GetGridFrame().get();
  }
  
private:
  Esbot * m_esbot;
};


class EsbotCarrotProxy: public CarrotProxy {
public:
  EsbotCarrotProxy(const Esbot * _esbot, bool _full_trace)
    : esbot(_esbot), full_trace(_full_trace)
  {}
  
  virtual const estar::carrot_trace * GetCarrotTrace() const {
    if(full_trace) trace = esbot->ComputeFullCarrot();
    else           trace = esbot->GetCarrotTrace();
    return trace.get();
  }
  
  virtual const sfl::GridFrame * GetGridFrame() const {
    boost::shared_ptr<const PNF> pnf(esbot->GetPNF());
    if( ! pnf) return 0;
    return pnf->GetGridFrame().get();
  }
  
  const Esbot * esbot;
  mutable shared_ptr<carrot_trace> trace;
  bool full_trace;
};


Esbot::
Esbot(boost::shared_ptr<RobotDescriptor> descriptor,
      const World & world)
  : RobotClient(descriptor, world, 2, true),
    m_radius(0.0),
    m_speed(0.0),
    m_grid_width(8),		// override with option pnf_grid_width
    m_grid_wdim(60),		// override with option pnf_grid_wdim
    m_goal(new Goal()),
    m_cheat(new CheatSheet(&world, GetServer())),
    m_carrot_trace(new carrot_trace()),
    m_pose(new Frame()),
    m_replan_request(false)
{
  shared_ptr<Hull> hull(CreateHull());
  for(HullIterator ih(*hull); ih.IsValid(); ih.Increment())
    AddLine(Line(ih.GetX0(), ih.GetY0(), ih.GetX1(), ih.GetY1()));
  const_cast<double &>(m_radius) = hull->CalculateRadius();
  
  string_to(descriptor->GetOption("pnf_grid_width"), m_grid_width);
  string_to(descriptor->GetOption("pnf_grid_wdim"), m_grid_wdim);
  
  if( ! string_to(descriptor->GetOption("pnf_enable_thread"), m_enable_thread))
    m_enable_thread = false;
  
  robox_parameters params(descriptor);
  
  m_front = DefineLidar(Frame(params.front_mount_x,
			      params.front_mount_y,
			      params.front_mount_theta),
			params.front_nscans,
			params.front_rhomax,
			params.front_phi0,
			params.front_phirange,
			params.front_channel)->GetScanner();
  m_rear = DefineLidar(Frame(params.rear_mount_x,
			     params.rear_mount_y,
			     params.rear_mount_theta),
		       params.rear_nscans,
		       params.rear_rhomax,
		       params.rear_phi0,
		       params.rear_phirange,
		       params.rear_channel)->GetScanner();

  m_drive = DefineDiffDrive(params.model_wheelbase, params.model_wheelradius);
  const RobotModel::Parameters modelParms(CreateRobotParameters(params));
  m_robotModel.reset(new RobotModel(modelParms, hull));
  m_motionController.reset(new MotionController(m_robotModel, GetHAL(),
						RWlock::Create("motor")));
  m_odometry.reset(new Odometry(GetHAL(), RWlock::Create("odometry")));
  m_multiscanner.reset(new Multiscanner(GetHAL()));
  m_dynamicWindow.reset(new LegacyDynamicWindow(params.dwa_dimension,
						params.dwa_grid_width,
						params.dwa_grid_height,
						params.dwa_grid_resolution,
						m_robotModel,
						params.dwa_alpha_distance,
						params.dwa_alpha_heading,
						params.dwa_alpha_speed,
						true));
  
  m_multiscanner->Add(m_front);
  m_multiscanner->Add(m_rear);
  
  CreateGfxStuff(descriptor->name);
}


void Esbot::
CreateGfxStuff(const std::string & name)
{
  AddDrawing(new GoalInstanceDrawing(name + "_goaldrawing", *m_goal));
  AddDrawing(new DWDrawing(name + "_dwdrawing", *m_dynamicWindow));
  AddDrawing(new ODrawing(name + "_dodrawing",
			  m_dynamicWindow->GetDistanceObjective(),
			  m_dynamicWindow));
  AddDrawing(new ODrawing(name + "_hodrawing",
			  m_dynamicWindow->GetHeadingObjective(),
			  m_dynamicWindow));
  AddDrawing(new ODrawing(name + "_sodrawing",
			  m_dynamicWindow->GetSpeedObjective(),
			  m_dynamicWindow));
  AddDrawing(new OdometryDrawing(name + "_odomdrawing",
				 *m_odometry,
				 m_robotModel->WheelBase() / 2));
  AddDrawing(new DODrawing(name + "_collisiondrawing",
			   m_dynamicWindow->GetDistanceObjective(),
			   m_dynamicWindow->GetHeadingObjective(),
			   m_dynamicWindow,
			   m_robotModel));
  AddDrawing(new PNFDrawing(name + "_global_risk",
			    this, PNFDrawing::GLOBAL, PNFDrawing::RISK));
  AddDrawing(new PNFDrawing(name + "_gframe_risk",
			    this, PNFDrawing::GFRAME, PNFDrawing::RISK));
  AddDrawing(new PNFDrawing(name + "_global_meta",
			    this, PNFDrawing::GLOBAL, PNFDrawing::META));
  AddDrawing(new PNFDrawing(name + "_gframe_meta",
			    this, PNFDrawing::GFRAME, PNFDrawing::META));
  AddDrawing(new PNFDrawing(name + "_global_value",
			    this, PNFDrawing::GLOBAL, PNFDrawing::VALUE));
  AddDrawing(new PNFDrawing(name + "_gframe_value",
			    this, PNFDrawing::GFRAME, PNFDrawing::VALUE));
  
  AddDrawing(new EstarDrawing(name + "_envdist", shared_ptr<PlanProxy>(new EnvdistProxy(this)), EstarDrawing::VALUE));
			      
  // XXX to do: magic numbers!!!
  AddDrawing(new CarrotDrawing(name + "_carrot",
	     shared_ptr<EsbotCarrotProxy>(new EsbotCarrotProxy(this, false)),
	     0));
  AddDrawing(new CarrotDrawing(name + "_fullcarrot",
	     shared_ptr<EsbotCarrotProxy>(new EsbotCarrotProxy(this, true)),
	     5));
  
  AddCamera(new StillCamera(name + "_dwcamera",
			    0,
			    0,
			    m_dynamicWindow->Dimension(),
			    m_dynamicWindow->Dimension(),
			    Instance<UniqueManager<Camera> >()));
  AddCamera(new OCamera(name + "_ocamera", *m_dynamicWindow));
  double a, b, c, d;
  m_dynamicWindow->GetDistanceObjective()->GetRange(a, b, c, d);
  AddCamera(new StillCamera(name + "_collisioncamera", a, b, c, d,
			    Instance<UniqueManager<Camera> >()));
  AddCamera(new PNFCamera(name + "_pnfcamera", this));
}


void Esbot::
InitPose(double x,
	 double y,
	 double theta)
{
  m_odometry->Init(Pose(x, y, theta));
}


void Esbot::
SetPose(double x,
	double y,
	double theta)
{
  m_odometry->Set(Pose(x, y, theta));
}


void Esbot::
GetPose(double & x,
	double & y,
	double & theta)
{
  x = m_pose->X();
  y = m_pose->Y();
  theta = m_pose->Theta();
}


shared_ptr<const Goal> Esbot::
GetGoal()
{
  return m_goal;
}


bool Esbot::
GoalReached()
{
  return m_goal->Reached(*m_pose, true);
}


void Esbot::
SetGoal(double timestep, const Goal & goal)
{
  PVDEBUG("%g   %g   %g   %g   %g\n", goal.X(), goal.Y(), goal.Theta(),
	 goal.Dr(), 180 * goal.Dtheta() / M_PI);
  m_goal->Set(goal);
  m_replan_request = true;
}


bool Esbot::
PrepareAction(double timestep)
{
  m_pose->Set(*m_odometry->Get());
  m_front->Update();
  m_rear->Update();

  if(m_replan_request){
    m_replan_request = false;
    
    m_pnf.reset(new PNF(m_pose->X(), m_pose->Y(), m_radius, m_speed,
			m_goal->X(), m_goal->Y(), m_goal->Dr(),
			m_grid_width, m_grid_wdim, m_enable_thread));
    PVDEBUG("static lines...\n");
    m_cheat->UpdateLines();
    bool ok(false);
    for(size_t il(0); il < m_cheat->line.size(); ++il)
      if(m_pnf->AddStaticLine(m_cheat->line[il].x0, m_cheat->line[il].y0,
			      m_cheat->line[il].x1, m_cheat->line[il].y1))
	ok = true;
    if( ! ok){
      cerr << __func__ << "(): oops AddStaticLine [not able to do without]\n";
      exit(EXIT_FAILURE);
    }
    m_cheat->UpdateDynobjs();
    PVDEBUG("%zu dynamic objects...\n", m_cheat->dynobj.size());
    ok = false;
    for(size_t ir(0); ir < m_cheat->dynobj.size(); ++ir)
      if(m_pnf->SetDynamicObject(ir, m_cheat->dynobj[ir].x,
				 m_cheat->dynobj[ir].y,
				 m_cheat->dynobj[ir].r, m_speed)){
	PVDEBUG("OK: [%zu] %g  %g  %g\n", ir, m_cheat->dynobj[ir].x,
	       m_cheat->dynobj[ir].y, m_cheat->dynobj[ir].r);
	ok = true;
      }
      else
	PVDEBUG("fail: [%zu] %g  %g  %g\n", ir, m_cheat->dynobj[ir].x,
	       m_cheat->dynobj[ir].y, m_cheat->dynobj[ir].r);
    if( ! ok){
      cerr << __func__ << "(): oops SetDynamicObject [cannot do without]\n";
      exit(EXIT_FAILURE);
    }
    m_pnf->StartPlanning();
    m_dynamicWindow->GoSlow();
    PVDEBUG("DONE\n");
  }
  
  static PNF::step_t prevstep(static_cast<PNF::step_t>(PNF::DONE + 1));
  
  const PNF::step_t step(m_pnf->GetStep(true));
  if(step != prevstep)
    cerr << "**************************************************\n"
	 << __func__ << "(): step " << prevstep << " ==> " << step << "\n";
  prevstep = step;
  
  m_carrot_trace->clear();
  
  // default: go straight for goal
  double carx(m_goal->X() - m_pose->X());
  double cary(m_goal->Y() - m_pose->Y());
  if(PNF::DONE == step){
    PVDEBUG("(PNF::DONE == step)\n");
    shared_ptr<const GridFrame> gframe(m_pnf->GetGridFrame());
    double robx(m_pose->X());
    double roby(m_pose->Y());
    PVDEBUG("global robot            : %g   %g\n", robx, roby);
    gframe->From(robx, roby);
    PVDEBUG("grid local robot        : %g   %g\n", robx, roby);
    const double carrot_distance(1.2); // XXX to do: magic numbers...
    const double carrot_stepsize(0.3);
    const size_t carrot_maxnsteps(30);
    const int result(m_pnf->GetFlow()->GetPNF().
		     TraceCarrot(robx, roby,
				 carrot_distance, carrot_stepsize,
				 carrot_maxnsteps, *m_carrot_trace, 0));
    if(0 <= result){
      if(1 == result)
	PVDEBUG("WARNING: carrot didn't reach distance %g\n", carrot_distance);
      PVDEBUG("carrot.value = %g\n", m_carrot_trace->back().value);
      if(m_carrot_trace->back().value <= 3 * carrot_stepsize){
	// could just as well simply keep (carx, cary) untouched, but
	// we want to see this adaption in the CarrotDrawing... his is
	// really "dumb" because we undo the same trasformations right
	// afterwards, but well.
	PVDEBUG("carrot on goal border, appending goal point to carrot");
	double foox(m_goal->X());
	double fooy(m_goal->Y());
	gframe->From(foox, fooy);
	m_carrot_trace->push_back(carrot_item(foox, fooy, 0, 0, 0, true));
      }
      carx = m_carrot_trace->back().cx;
      cary = m_carrot_trace->back().cy;
      PVDEBUG("grid local carrot       : %g   %g\n", carx, cary);
      gframe->To(carx, cary);
      PVDEBUG("global carrot           : %g   %g\n", carx, cary);
      carx -= m_pose->X();
      cary -= m_pose->Y();
      PVDEBUG("global delta carrot     : %g   %g\n", carx, cary);
    }
    else
      PVDEBUG("FAILED compute_carrot()\n");
  }
  else
    PVDEBUG("no local goal can be determined\n");    
  m_pose->RotateFrom(carx, cary);
  PVDEBUG("robot local delta carrot: %g   %g\n", carx, cary);
  
  // pure rotation hysteresis
  static const double start_aiming(10 * M_PI / 180); // to do: magic numbers
  static const double start_homing(2 * M_PI / 180); // to do: magic numbers
  const double dhead(atan2(cary, carx));
  if(sfl::absval(dhead) < start_homing)
    m_dynamicWindow->GoFast();
  else if(sfl::absval(dhead) > start_aiming)
    m_dynamicWindow->GoSlow();
  
  shared_ptr<const Scan> scan(m_multiscanner->CollectScans());
  
  double qdl, qdr;
  m_motionController->GetCurrentAct(qdl, qdr);
  m_dynamicWindow->Update(qdl, qdr, timestep, carx, cary, scan);
  if( ! m_dynamicWindow->OptimalActuators(qdl, qdr)){
    qdl = 0;
    qdr = 0;
  }
  
  m_motionController->ProposeActuators(qdl, qdr);
  m_motionController->Update(timestep);
  m_odometry->Update();

  return true;
}


RobotModel::Parameters
CreateRobotParameters(expo_parameters & params)
{
  return RobotModel::
    Parameters(params.model_security_distance,
	       params.model_wheelbase,
	       params.model_wheelradius,
	       params.model_qd_max,
	       params.model_qdd_max,
	       params.model_sd_max,
	       params.model_thetad_max,
	       params.model_sdd_max,
	       params.model_thetadd_max);
}


shared_ptr<Hull>
CreateHull()
{
  shared_ptr<Hull> hull(new Hull());
  static const double octoSmall = 0.178;
  static const double octoBig = 0.430;
  
  Polygon outline;		// temporary
  outline.AddPoint( octoBig  , octoSmall);
  outline.AddPoint( octoSmall, octoBig);
  outline.AddPoint(-octoSmall, octoBig);
  outline.AddPoint(-octoBig  , octoSmall);
  outline.AddPoint(-octoBig  ,-octoSmall);
  outline.AddPoint(-octoSmall,-octoBig);
  outline.AddPoint( octoSmall,-octoBig);
  outline.AddPoint( octoBig  ,-octoSmall);
  hull->AddPolygon(outline);
  
  return hull;
}


boost::shared_ptr<carrot_trace> Esbot::
ComputeFullCarrot() const
{
  // can only compute full carrot if the partial one exists...
  if((!m_carrot_trace) || m_carrot_trace->empty()){
    PVDEBUG("m_carrot_trace.empty()\n");
    return shared_ptr<carrot_trace>();
  }
  
  shared_ptr<const GridFrame> gframe(m_pnf->GetGridFrame());
  double robx(m_pose->X());
  double roby(m_pose->Y());
  gframe->From(robx, roby);
  const double distance(sqrt(sqr(m_goal->X() - m_pose->X())
			     + sqr(m_goal->Y() - m_pose->Y()))
			- 0.5 * m_goal->Dr());
  const double stepsize(0.2);	// magic numbers stink
  const size_t maxnsteps(static_cast<size_t>(ceil(2 * distance / stepsize)));
  shared_ptr<carrot_trace> trace(new carrot_trace);
  const int result(m_pnf->GetFlow()->GetPNF().
		   TraceCarrot(robx, roby,
			       distance, stepsize, maxnsteps, *trace, 0));
  PVDEBUG("d: %g   s: %g   n: %lu   result: %d\n",
	  distance, stepsize, maxnsteps, result);
  
  if(0 <= result)
    return trace;
  
  return shared_ptr<carrot_trace>();
}