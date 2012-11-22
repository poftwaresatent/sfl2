/* 
 * Copyright (C) 2012 Roland Philippsen. All rights reserved.
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

#ifndef NPM_EXT_PYBOT_HPP
#define NPM_EXT_PYBOT_HPP

#include <npm/RobotClient.hpp>
#include <fpplib/factory.hpp>


namespace npm {
  
  
  class PyBot
    : public RobotClient
  {
  public:
    PyBot(std::string const &name,
	  PyObject *pyrob,    // passed as first argument to the Python update callable
	  PyObject *pyinit,   // must be a callable (ie. pass PyCallable_Check)
	  PyObject *pyupdate, // must be a callable (ie. pass PyCallable_Check)
	  PyObject *pyparms); // must be a mapping (ie. pass PyMapping_Check)
    virtual ~PyBot();
    
    virtual bool Initialize(RobotServer &server);
    virtual bool PrepareAction(double timestep);
    virtual void InitPose(sfl::Pose const &pose) {}
    virtual void SetPose(sfl::Pose const &pose) {}
    virtual bool GetPose(sfl::Pose &pose) { return false; }
    virtual void SetGoal(double timestep, const sfl::Goal & goal);
    virtual bool GetGoal(sfl::Goal &goal);
    virtual bool GoalReached();
    
  private:
    RobotServer *m_server;
    sfl::Goal m_goal;
    
    PyObject *m_pyrob;
    PyObject *m_pyinit;
    PyObject *m_pyupdate;
    PyObject *m_pyparms;
  };
  
  
  class PyBotCreator
    : public fpplib::Creator<PyBot>
  {
  public:
    bool init(std::string const &module,
	      std::string const &createfunc);
    
    virtual PyBot * create(std::string const & instance_name);
    
  private:
    PyObject *m_module;
    PyObject *m_createfunc;
  };
  
}

#endif
