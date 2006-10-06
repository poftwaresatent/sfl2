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
#include "PNFCamera.hpp"
#include "CarrotDrawing.hpp"
#include <npm/robox/OCamera.hpp>
#include <npm/robox/ODrawing.hpp>
#include <npm/robox/DODrawing.hpp>
#include <npm/robox/DWDrawing.hpp>
#include <npm/common/Lidar.hpp>
#include <npm/common/GoalInstanceDrawing.hpp>
#include <npm/common/OdometryDrawing.hpp>
#include <npm/common/StillCamera.hpp>
#include <npm/common/HALProxy.hpp>
#include <npm/common/CheatSheet.hpp>
#include <sfl/api/Odometry.hpp>
#include <sfl/api/Multiscanner.hpp>
#include <sfl/api/DiffDrive.hpp>
#include <sfl/api/MotionController.hpp>
#include <sfl/dwa/DynamicWindow.hpp>
#include <sfl/gplan/GridFrame.hpp>
#include <estar/Grid.hpp>
#include <estar/Facade.hpp>
#include <estar/dump.hpp>
#include <pnf/Flow.hpp>
#include <iostream>
#include <set>

// debugging
#include <estar/util.hpp>
#define PDEBUG PDEBUG_ERR
#define PVDEBUG PDEBUG_OFF


using sfl::Frame;
using sfl::Hull;
using sfl::Polygon;
using sfl::Pose;
using sfl::Goal;
using sfl::RobotModel;
using sfl::DiffDrive;
using sfl::DynamicWindow;
using sfl::Odometry;
using sfl::Multiscanner;
using sfl::MotionController;
using sfl::GridFrame;
using sfl::GlobalScan;
using sfl::sqr;
using sfl::absval;
using estar::Grid;
using estar::GridNode;
using estar::vertex_t;
using estar::minval;
using estar::dump_grid_range_highlight;
using pnf::Flow;
using boost::shared_ptr;
using std::string;
using std::cerr;
using std::set;
using std::make_pair;


static RobotModel::Parameters
CreateRobotParameters(shared_ptr<const DiffDrive> dd);

static shared_ptr<Hull> CreateHull();  


Esbot::
Esbot(boost::shared_ptr<RobotDescriptor> descriptor,
      const World & world,
      double timestep)
  : Robot(descriptor, world, timestep, true),
    m_grid_width(8),		// TO DO: no magic numbers
    m_grid_wdim(30),		// TO DO: no magic numbers
    m_cheat(new CheatSheet(&world, this)),
    m_carrot_trace(new carrot_trace_t)
{
  shared_ptr<Hull> hull(CreateHull());
  for(size_t ip(0); ip < hull->GetNPolygons(); ++ip){
    shared_ptr<const Polygon> poly(hull->GetPolygon(ip));
    for(size_t il(0); il < poly->GetNPoints(); ++il)
      AddLine(*poly->GetLine(il));
  }
  const_cast<double &>(m_radius) = hull->CalculateRadius();
  
  static const double LIDAROFFSET = 0.15;
  m_front = DefineLidar(Frame(LIDAROFFSET, 0, 0),    "front");
  m_rear = DefineLidar(Frame(-LIDAROFFSET, 0, M_PI), "rear");

  static const double WHEELBASE = 0.521;
  static const double WHEELRADIUS = 0.088;
  m_drive = DefineDiffDrive(WHEELBASE, WHEELRADIUS);

  const RobotModel::Parameters modelParms(CreateRobotParameters(m_drive));
  m_robotModel.reset(new RobotModel(timestep, modelParms, hull));
  const_cast<double &>(m_speed) = m_robotModel->SdMax();
  
  m_motionController.reset(new MotionController(*m_robotModel,
						*m_drive));
  m_odometry.reset(new Odometry(GetHALProxy()));
  m_multiscanner.reset(new Multiscanner());
  
  const string dwa_mode(descriptor->GetOption("dwa_mode"));
  if(dwa_mode == "full")
    m_dynamicWindow.reset(new DynamicWindow(41,	// dwa dimension
					    2.2, // dwa grid width
					    1.5, // dwa grid height
					    0.03, // dwa grid resolution
					    *m_robotModel,
					    *m_motionController,
					    0.5, // dwa alpha distance
					    0.1, // dwa alpha heading
					    0.1, // dwa alpha speed
					    true)); // auto_init
  else if(dwa_mode == "light")
    m_dynamicWindow.reset(new DynamicWindow(21,	// dwa dimension
					    2.2, // dwa grid width
					    1.5, // dwa grid height
					    0.1, // dwa grid resolution
					    *m_robotModel,
					    *m_motionController,
					    0.5, // dwa alpha distance
					    0.1, // dwa alpha heading
					    0.1, // dwa alpha speed
					    true)); // auto_init
  else{
    cerr << "ERROR in Esbot ctor, invalid option dwa_mode=\""
	 << dwa_mode << "\"\n"
	 << "  use \"full\" or \"light\"\n";
    exit(EXIT_FAILURE);
  }
  
  m_multiscanner->Add(m_front);
  m_multiscanner->Add(m_rear);
  
  CreateGfxStuff(descriptor->GetName());
}


void Esbot::
CreateGfxStuff(const std::string & name)
{
  AddDrawing(new GoalInstanceDrawing(name + "_goaldrawing", m_goal));
  AddDrawing(new DWDrawing(name + "_dwdrawing", *m_dynamicWindow));
  AddDrawing(new ODrawing(name + "_dodrawing",
			  m_dynamicWindow->GetDistanceObjective(),
			  *m_dynamicWindow));
  AddDrawing(new ODrawing(name + "_hodrawing",
			  m_dynamicWindow->GetHeadingObjective(),
			  *m_dynamicWindow));
  AddDrawing(new ODrawing(name + "_sodrawing",
			  m_dynamicWindow->GetSpeedObjective(),
			  *m_dynamicWindow));
  AddDrawing(new OdometryDrawing(name + "_odomdrawing",
				 *m_odometry,
				 m_robotModel->WheelBase() / 2));
  AddDrawing(new DODrawing(name + "_collisiondrawing",
			   m_dynamicWindow->GetDistanceObjective(),
			   *m_dynamicWindow,
			   *m_robotModel));
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
  AddDrawing(new CarrotDrawing(name + "_carrotdrawing", this, true, false));
  AddDrawing(new CarrotDrawing(name + "_fullcarrotdrawing", this, true, true));
  AddDrawing(new CarrotDrawing(name + "_localcarrotdrawing",
			       this, false, false));
  
  AddCamera(new StillCamera(name + "_dwcamera",
			    0,
			    0,
			    m_dynamicWindow->Dimension(),
			    m_dynamicWindow->Dimension(),
			    true));
  AddCamera(new OCamera(name + "_ocamera", *m_dynamicWindow));
  double a, b, c, d;
  m_dynamicWindow->GetDistanceObjective().GetRange(a, b, c, d);
  AddCamera(new StillCamera(name + "_collisioncamera", a, b, c, d, true));
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
  x = m_odometry->Get().X();
  y = m_odometry->Get().Y();
  theta = m_odometry->Get().Theta();
}


const Goal & Esbot::
GetGoal()
{
  return m_goal;
}


bool Esbot::
GoalReached()
{
  return m_goal.Reached(m_odometry->Get(), true);
}


void Esbot::
SetGoal(const Goal & goal)
{
  PDEBUG("%g   %g   %g   %g   %g\n", goal.X(), goal.Y(), goal.Theta(),
	 goal.Dr(), 180 * goal.Dtheta() / M_PI);
  m_goal.Set(goal);
  const Pose pose(m_odometry->Get());
  m_pnf.reset();
  m_pnf.reset(new PNF(pose.X(), pose.Y(), m_radius, m_speed,
		      goal.X(), goal.Y(), goal.Dr(),
		      m_grid_width, m_grid_wdim));
  PDEBUG("static lines...\n");
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
  PDEBUG("dynamic objects...\n");
  m_cheat->UpdateDynobjs();
  ok = false;
  for(size_t ir(0); ir < m_cheat->dynobj.size(); ++ir)
    if(m_pnf->SetDynamicObject(ir, m_cheat->dynobj[ir].x,
			       m_cheat->dynobj[ir].y,
			       m_cheat->dynobj[ir].r, m_speed))
      ok = true;
  if( ! ok){
    cerr << __func__ << "(): oops SetDynamicObject [not able to do without]\n";
    exit(EXIT_FAILURE);
  }
  m_pnf->StartPlanning();
  m_dynamicWindow->GoSlow();
  PDEBUG("DONE\n");
}


void Esbot::
PrepareAction()
{
  m_front->Update();
  m_rear->Update();
  
  static PNF::step_t prevstep(static_cast<PNF::step_t>(PNF::DONE + 1));
  
  const PNF::step_t step(m_pnf->GetStep());
  if(step != prevstep)
    cerr << "**************************************************\n"
	 << __func__ << "(): step " << prevstep << " ==> " << step << "\n";
  prevstep = step;
  
  m_carrot_trace->clear();
  m_pose.Set(m_odometry->Get());
  const double goaldist(sqrt(sqr(m_pose.X() - m_goal.X())
			     + sqr(m_pose.X() - m_goal.X())));
  // TO DO: the smell of magic numbers...
  const double carrot_distance(minval(goaldist, 1.5));
  
  // default: go straight for goal
  double carx(m_goal.X() - m_pose.X());
  double cary(m_goal.Y() - m_pose.Y());
  if(goaldist <= carrot_distance)
    PVDEBUG("(goaldist <= carrot_distance)\n");
  else if(PNF::DONE == step){
    PVDEBUG("(PNF::DONE == step)\n");
    const sfl::Frame & gframe(m_pnf->GetGridFrame()->GetFrame());
    double robx(m_pose.X());
    double roby(m_pose.Y());
    PVDEBUG("global robot            : %g   %g\n", robx, roby);
    gframe.From(robx, roby);
    PVDEBUG("grid local robot        : %g   %g\n", robx, roby);
    const double carrot_stepsize(0.3);
    const size_t carrot_maxnsteps(30);
    const int
      result(estar::compute_carrot(m_pnf->GetFlow()->GetPNF(), robx, roby,
				   carrot_distance, carrot_stepsize,
				   carrot_maxnsteps, carx, cary,
				   m_carrot_trace.get()));
    if(0 <= result){
      if(1 == result)
	PVDEBUG("WARNING: carrot didn't reach distance %g\n", carrot_distance);
      PVDEBUG("grid local carrot       : %g   %g\n", carx, cary);
      gframe.To(carx, cary);
      PVDEBUG("global carrot           : %g   %g\n", carx, cary);
      carx -= m_pose.X();
      cary -= m_pose.Y();
      PVDEBUG("global delta carrot     : %g   %g\n", carx, cary);
    }
    else{
      PDEBUG("FAILED compute_carrot()\n");
      // paranoia in case carx, cary got modified anyways
      carx = m_goal.X() - m_pose.X();
      cary = m_goal.Y() - m_pose.Y();    
    }
  }
  else
    PVDEBUG("no local goal can be determined\n");    
  m_pose.RotateFrom(carx, cary);
  PVDEBUG("robot local delta carrot: %g   %g\n", carx, cary);
  
  // pure rotation hysteresis
  static const double start_aiming(10 * M_PI / 180); // to do: magic numbers
  static const double start_homing(2 * M_PI / 180); // to do: magic numbers
  const double dhead(atan2(cary, carx));
  if(absval(dhead) < start_homing)
    m_dynamicWindow->GoFast();
  else if(absval(dhead) > start_aiming)
    m_dynamicWindow->GoSlow();
  
  shared_ptr<const GlobalScan>
    global_scan(m_multiscanner->CollectGlobalScans(m_pose));
  m_dynamicWindow->Update(carx, cary, global_scan);
  double qdl, qdr;
  if( ! m_dynamicWindow->OptimalActuators(qdl, qdr)){
    qdl = 0;
    qdr = 0;
  }
  
  m_motionController->ProposeActuators(qdl, qdr);
  m_motionController->Update();
  m_odometry->Update();
}


RobotModel::Parameters
CreateRobotParameters(shared_ptr<const DiffDrive> dd)
{
  return RobotModel::
    Parameters(0.02, // safety distance
	       dd->WheelBase(), // wheel base
	       dd->WheelRadius(), // wheel radius
	       6.5, // qd max
	       6.5, // qdd max
	       0.5, // sd max
	       2.0, // thetad max
	       0.75*dd->WheelRadius()*6.5, // sdd max
	       1.5*dd->WheelRadius()*6.5/dd->WheelBase() // thetadd max
	       );
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


boost::shared_ptr<Esbot::carrot_trace_t> Esbot::
ComputeFullCarrot() const
{
  // can only compute full carrot if the partial one exists...
  if((!m_carrot_trace) || m_carrot_trace->empty()){
    PVDEBUG("m_carrot_trace.empty()\n");
    return shared_ptr<carrot_trace_t>();
  }
  
  const sfl::Frame & gframe(m_pnf->GetGridFrame()->GetFrame());
  double robx(m_pose.X());
  double roby(m_pose.Y());
  gframe.From(robx, roby);
  const double distance(sqrt(sqr(m_goal.X() - m_pose.X())
			     + sqr(m_goal.Y() - m_pose.Y()))
			- 0.5 * m_goal.Dr());
  const double stepsize(0.2);	// magic numbers stink
  const size_t maxnsteps(static_cast<size_t>(ceil(2 * distance / stepsize)));
  double carx, cary;
  shared_ptr<carrot_trace_t> trace(new carrot_trace_t);
  const int
    result(estar::compute_carrot(m_pnf->GetFlow()->GetPNF(),
				 robx, roby,
				 distance, stepsize, maxnsteps,
				 carx, cary, trace.get()));
  PVDEBUG("d: %g   s: %g   n: %lu   result: %d\n",
	  distance, stepsize, maxnsteps, result);
  
  if(0 <= result)
    return trace;
  
  return shared_ptr<carrot_trace_t>();
}
