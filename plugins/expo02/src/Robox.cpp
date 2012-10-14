/* 
 * Copyright (C) 2004
 * Swiss Federal Institute of Technology, Lausanne. All rights reserved.
 * 
 * Author: Roland Philippsen <roland dot philippsen at gmx dot net>
 *         Autonomous Systems Lab <http://asl.epfl.ch/>
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

#include "Robox.hpp"
#include "robox_parameters.hpp"
#include "VisualRobox.hpp"
#include "../common/World.hpp"
#include "../common/HAL.hpp"
#include "../common/RobotDescriptor.hpp"
#include "../common/Lidar.hpp"
#include "../common/pdebug.hpp"
#include "../common/Manager.hpp"
#include <sfl/util/strutil.hpp>
#include <sfl/api/Odometry.hpp>
#include <sfl/api/Multiscanner.hpp>
#include <sfl/api/RobotModel.hpp>
#include <sfl/api/Goal.hpp>
#include <sfl/dwa/DynamicWindow.hpp>
#include <sfl/dwa/DistanceObjective.hpp>
#include <sfl/dwa/SpeedObjective.hpp>
#include <sfl/dwa/HeadingObjective.hpp>
#include <sfl/expo/expo_parameters.h>
#include <sfl/expo/MotionPlanner.hpp>
#include <sfl/expo/MotionController.hpp>
#include <sfl/expo/Robox.hpp>


using namespace npm;
using namespace sfl;
using namespace boost;
using namespace std;


namespace local {
  
  class MPKeyListener: public KeyListener {
  public:
    MPKeyListener(shared_ptr<expo::MotionPlanner> _mp)
      : mp(_mp), stopped(false) {}
    
    virtual void KeyPressed(unsigned char key)
    {
      if('m' != key)
	return;
      if(stopped){
	stopped = false;
	mp->ManualResume();
      }
      else{
	stopped = true;
	mp->ManualStop();
      }
    }
    
    shared_ptr<expo::MotionPlanner> mp;
    bool stopped;
  };
  
  
  class NGKeyListener: public KeyListener {
  public:
    NGKeyListener(): next_goal(false) {}
    
    virtual void KeyPressed(unsigned char key)
    { if('n' == key) next_goal = true; }
    
    bool next_goal;
  };
  
}


scanner_desc_s::
scanner_desc_s()
  : nscans(181),
    mount_x(0.15),
    mount_y(0),
    mount_theta(0),
    rhomax(8),
    phi0(-M_PI/2),
    phirange(M_PI)
{
}


Robox::
Robox(boost::shared_ptr<npm::RobotDescriptor> descriptor,
      const npm::World & world,
      boost::shared_ptr<sfl::Hull> hull,
      std::map<int, scanner_desc_s> const & scanners)
  : RobotClient(descriptor, world, 2, true),
    m_ngkl(new local::NGKeyListener())
{
  boost::shared_ptr<sfl::Multiscanner> mscan(new Multiscanner(GetHAL()));
  for (std::map<int, scanner_desc_s>::const_iterator iscan(scanners.begin());
       iscan != scanners.end(); ++iscan) {
    mscan->Add(DefineLidar(Frame(iscan->second.mount_x,
				 iscan->second.mount_y,
				 iscan->second.mount_theta),
			   iscan->second.nscans,
			   iscan->second.rhomax,
			   iscan->second.phi0,
			   iscan->second.phirange,
			   iscan->first)->GetScanner());
  }
  
  expo_parameters params(descriptor);
  m_drive = DefineDiffDrive(params.model_wheelbase, params.model_wheelradius);
  
  m_imp.reset(new npm::VisualRobox(descriptor->name, params, hull, GetHAL(), mscan));
  
  for (HullIterator ih(*m_imp->hull); ih.IsValid(); ih.Increment()) {
    AddLine(Line(ih.GetX0(), ih.GetY0(), ih.GetX1(), ih.GetY1()));
    PDEBUG("line %05.2f %05.2f %05.2f %05.2f\n",
	   ih.GetX0(), ih.GetY0(), ih.GetX1(), ih.GetY1());
  }
  
  world.AddKeyListener(m_ngkl);
  shared_ptr<KeyListener> listener(new local::MPKeyListener(m_imp->motionPlanner));
  world.AddKeyListener(listener);
}


Robox::
Robox(shared_ptr<RobotDescriptor> descriptor,
      const World & world,
      boost::shared_ptr<sfl::Hull> hull)
  : RobotClient(descriptor, world, 2, true),
    m_ngkl(new local::NGKeyListener())
{
  robox_parameters params(descriptor);
  
  boost::shared_ptr<sfl::Scanner>
    front = DefineLidar(Frame(params.front_mount_x,
			      params.front_mount_y,
			      params.front_mount_theta),
			params.front_nscans,
			params.front_rhomax,
			params.front_phi0,
			params.front_phirange,
			params.front_channel)->GetScanner();
  boost::shared_ptr<sfl::Scanner>
    rear = DefineLidar(Frame(params.rear_mount_x,
			     params.rear_mount_y,
			     params.rear_mount_theta),
		       params.rear_nscans,
		       params.rear_rhomax,
		       params.rear_phi0,
		       params.rear_phirange,
		       params.rear_channel)->GetScanner();
  boost::shared_ptr<sfl::Multiscanner> mscan(new Multiscanner(GetHAL()));
  mscan->Add(front);
  mscan->Add(rear);
  
  m_drive = DefineDiffDrive(params.model_wheelbase, params.model_wheelradius);
  
  m_imp.reset(new npm::VisualRobox(descriptor->name, params, hull, GetHAL(), mscan));
  
  for (HullIterator ih(*m_imp->hull); ih.IsValid(); ih.Increment()) {
    AddLine(Line(ih.GetX0(), ih.GetY0(), ih.GetX1(), ih.GetY1()));
    PDEBUG("line %05.2f %05.2f %05.2f %05.2f\n",
	   ih.GetX0(), ih.GetY0(), ih.GetX1(), ih.GetY1());
  }
  
  world.AddKeyListener(m_ngkl);
  shared_ptr<KeyListener> listener(new local::MPKeyListener(m_imp->motionPlanner));
  world.AddKeyListener(listener);
}


Robox * Robox::
Create(shared_ptr<RobotDescriptor> descriptor, const World & world)
{
  Robox * robox(new Robox(descriptor, world, expo::Robox::CreateDefaultHull()));
  if( ! robox->StartThreads()){
    delete robox;
    return 0;
  }
  return robox;
}


Robox * Robox::
CreateCustom(shared_ptr<RobotDescriptor> descriptor, const World & world)
{
  try {
    std::map<int, scanner_desc_s> scanners;
    boost::scoped_ptr<sfl::Polygon> polygon;
    boost::shared_ptr<sfl::Hull> hull;
    typedef RobotDescriptor::custom_line_t::const_iterator line_it_t;
    line_it_t iline(descriptor->GetCustomLines().begin());
    line_it_t endline(descriptor->GetCustomLines().end());
    for (/**/; iline != endline; ++iline) {
      std::string lno("line " + sfl::to_string(iline->first) + ": ");
      std::istringstream is(iline->second);
      PDEBUG ("line %d: %s\n", iline->first, iline->second.c_str());
      string token;
      is >> token;
      if (token == "scanner") {
	int channel;
	string keyword;
	is >> channel >> keyword;
	if ( ! is) {
	  throw std::runtime_error(lno + "invalid channel or keyword for scanner");
	}
	if (keyword == "mount") {
	  double xx, yy, theta;
	  is >> xx >> yy >> theta;
	  if ( ! is) {
	    throw std::runtime_error(lno + "invalid mount for scanner");
	  }
	  scanners[channel].mount_x = xx;
	  scanners[channel].mount_y = yy;
	  scanners[channel].mount_theta = theta;
	  PDEBUG ("scanner[%d].mount = %g %g %g\n", channel, xx, yy, theta);
	}
	else if (keyword == "nscans") {
	  int nscans;
	  is >> nscans;
	  if (( ! is) || (0 >= nscans)) {
            throw std::runtime_error(lno + "invalid nscans for scanner");
          }
	  scanners[channel].nscans = nscans;
	  PDEBUG ("scanner[%d].nscans = %d\n", channel, nscans);
	}
	else if (keyword == "rhomax") {
          double rhomax;
          is >> rhomax;
          if (( ! is) || (0 >= rhomax)) {
            throw std::runtime_error(lno + "invalid rhomax for scanner");
          }
          scanners[channel].rhomax = rhomax;
	  PDEBUG ("scanner[%d].rhomax = %g\n", channel, rhomax);
        }
	else if (keyword == "phi") {
          double phi0, phirange;
          is >> phi0 >> phirange;
          if ( ! is) {
            throw std::runtime_error(lno + "invalid phi0 or phirange for scanner");
          }
          scanners[channel].phi0 = phi0;
          scanners[channel].phirange = phirange;
	  PDEBUG ("scanner[%d].phi = %g %g\n", channel, phi0, phirange);
        }
	else {
	  throw std::runtime_error(lno + "invalid keyword `" + keyword + "' for scanner");
	}
      } // endif "scanner"
      else if (token == "hull") {
	string keyword;
	is >> keyword;
	if ( ! is) {
          throw std::runtime_error(lno + "invalid keyword for hull");
        }
	if ((keyword == "point") || (keyword == "points")) {
	  if ( ! polygon) {
	    polygon.reset(new sfl::Polygon());
	  }
	  double xx, yy;
	  while (is >> xx >> yy) {
	    polygon->AddPoint(xx, yy);
	    PDEBUG ("hull point %g %g\n", xx, yy);
	  }
	}
	else if (keyword == "break") {
	  if (polygon) {
	    if ( ! hull) {
	      hull.reset(new sfl::Hull());
	    }
	    hull->AddPolygon(*polygon);
	    polygon.reset();
	  }
	  PDEBUG ("hull break\n");
	}
	else {
	  throw std::runtime_error(lno + "invalid keyword `" + keyword + "' for hull");
	}
      } // endif "hull"
      else if (token == "diffdrive") {
	double wheelbase, wheelradius;
	is >> wheelbase >> wheelradius;
	if ( ! is) {
	  throw std::runtime_error(lno + "invalid diffdrive");
	}
	descriptor->SetOption("model_wheelbase", sfl::to_string(wheelbase));
	descriptor->SetOption("model_wheelradius", sfl::to_string(wheelradius));
	PDEBUG ("diffdrive %g %g\n", wheelbase, wheelradius);
      } // endif "diffdrive"
      else {
	throw std::runtime_error(lno + "invalid token `" + token + "'");
      }
    } // end for line
    
    // make sure we have at least one laser scanner
    if (scanners.empty()) {
      scanners[0] = scanner_desc_s();
      PDEBUG ("added default scanner\n");
    }
    
    // make sure we finalize the last (possibly only) polygon
    if (polygon) {
      if ( ! hull) {
	hull.reset(new sfl::Hull());
      }
      hull->AddPolygon(*polygon);
      PDEBUG ("finalized hull\n");
    }
    
    // make sure we have a hull
    if ( ! hull) {
      hull = expo::Robox::CreateDefaultHull();
      PDEBUG ("set default hull\n");
    }
    
    Robox * robox(new Robox(descriptor, world, hull, scanners));
    if( ! robox->StartThreads()){
      std::cerr << "ERROR in Robox::CreateCustom(): failed to StartThreads()\n";
      delete robox;
      return 0;
    }
    return robox;
    
  }
  catch (std::runtime_error const & ee) {
    std::cerr << "ERROR in Robox::CreateCustom(): " << ee.what() << "\n";
    return 0;
  }
  return 0;			// we never get here though.
}


void Robox::
InitPose(double x,
	 double y,
	 double theta)
{
  m_imp->odometry->Init(Pose(x, y, theta));
}


void Robox::
SetPose(double x,
	double y,
	double theta)
{
  m_imp->odometry->Set(Pose(x, y, theta));
}


void Robox::
GetPose(double & x,
	double & y,
	double & theta)
{
  shared_ptr<const Pose> pose(m_imp->odometry->Get());
  x = pose->X();
  y = pose->Y();
  theta = pose->Theta();
}


shared_ptr<const Goal> Robox::
GetGoal()
{
  return shared_ptr<const Goal>(new Goal(m_imp->GetGoal()));
}


bool Robox::
StartThreads()
{
  shared_ptr<RobotDescriptor> descriptor(GetDescriptor());
  if ((descriptor->GetOption("front_thread") != "")
      || (descriptor->GetOption("rear_thread") != "")
      || (descriptor->GetOption("mc_thread") != "")
      || (descriptor->GetOption("odo_thread") != "")
      || (descriptor->GetOption("bband_thread") != ""))
    cerr << "Robox::StartThreads(): threading was never really working, so it was kicked out.\n"
	 << "  the options front_thread, rear_thread, mc_thread, odo_thread, and bband_thread\n"
	 << "  are simply being ignored. Have  anice day.\n";
  return true;  
}


bool Robox::
GoalReached()
{
  if(m_ngkl->next_goal){
    m_ngkl->next_goal = false;
    return true;
  }
  return m_imp->motionPlanner->GoalReached();
}


void Robox::
SetGoal(double timestep, const Goal & goal)
{
  m_imp->SetGoal(timestep, goal);
}


bool Robox::
PrepareAction(double timestep)
{
  try {
    m_imp->Update(timestep);
  }
  catch (runtime_error ee) {
    cerr << "Robox::PrepareAction(): m_imp->Update() failed with exception\n"
	 << ee.what() << "\n";
    return false;
  }
  return true;
}
