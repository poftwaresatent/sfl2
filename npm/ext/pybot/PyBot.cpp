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

#include <Python.h>
#include "PyBot.hpp"
#include "pynpm.hpp"
#include <npm/HAL.hpp>
#include <npm/RobotServer.hpp>
#include <sfl/api/Goal.hpp>

using namespace sfl;
using namespace boost;
using namespace std;


namespace npm {
  
  PyBot::
  PyBot(std::string const &name,
	PyObject *pyrob,
	PyObject *pyinit,
	PyObject *pyupdate,
	PyObject *pyparms)
    : RobotClient(name),
      m_server(0),
      m_pyrob(pyrob),
      m_pyinit(pyinit),
      m_pyupdate(pyupdate),
      m_pyparms(pyparms)
  {
  }
  
  
  PyBot::
  ~PyBot()
  {
    Py_XDECREF(m_pyrob);
    Py_XDECREF(m_pyinit);
    Py_XDECREF(m_pyupdate);
    Py_XDECREF(m_pyparms);
  }
  
  
  bool PyBot::
  Initialize(RobotServer &server)
  {
    if ( !RobotClient::Initialize(server)) {
      return false;
    }
    
    m_server = &server;		// as a "cheat" so we don't need localization etc
    PyObject *pyserver = pynpm::createServer(&server);
    if (!pyserver) {
      cerr << "npm::PyBot::Initialize(): failed to create pynpm RobotServer proxy\n";
      return false;
    }
    
    PyObject *arg = Py_BuildValue("(o)", pyserver);
    Py_DECREF(pyserver);
    if (!arg) {
      cerr << "npm::PyBot::Initialize(): failed to create Python arg\n";
      return false;
    }
    
    PyObject *res = PyObject_CallObject(m_pyinit, arg);
    Py_DECREF(arg);
    if (!res) {
      PyErr_Print();
      cerr << "npm::PyBot::Initialize(): failed to initialize Python robot\n";
      return false;
    }
    Py_DECREF(res); // ignore the result, Python robots raise exceptions in case of problems
    
    return true;
  }
  
  
  bool PyBot::
  PrepareAction(double timestep)
  {
    const Frame & pose(m_server->GetTruePose());
    
    PyObject *arg =
      Py_BuildValue("{'true_pose': (ddd),"
		    " 'goal': (ddddd)}",
		    pose.X(), pose.Y(), pose.Theta(),
		    m_goal.X(), m_goal.Y(), m_goal.Theta(), m_goal.Dr(), m_goal.Dtheta());
    if (NULL == arg) {
      PyErr_Print();
      cerr << "npm::PyBot::PrepareAction(): failed to build argument tuple\n";
      return false;
    }
    
    PyObject *res = PyObject_CallObject(m_pyupdate, arg);
    Py_DECREF(arg);
    if (res == NULL) {
      PyErr_Print();
      cerr << "npm::PyBot::PrepareAction(): call to Python update function failed\n";
      return false;
    }
    
    if (!PySequence_Check(res)) {
      cerr << "npm::PyBot::PrepareAction(): Python update function did not return a sequence\n";
      Py_DECREF(res);
      return false;
    }
    
    Py_ssize_t const len = PySequence_Length(res);
    PyObject *seq = PySequence_Fast(res, "npm::PyBot::PrepareAction(): bug? expected a Python sequence");
    Py_DECREF(res);
    if (!seq) {
      PyErr_Print();
      cerr << "npm::PyBot::PrepareAction(): bug? thought I had checked for a Python sequence already...\n";
      return false;
    }
    
    double qd[len];
    for (Py_ssize_t ii = 0; ii < len; ++ii) {
      qd[ii] = PyFloat_AsDouble(PySequence_Fast_GET_ITEM(seq, ii));
      if (PyErr_Occurred()) {
	PyErr_Print();
	cerr << "npm::PyBot::PrepareAction(): Python update function must return a sequence of numbers.\n";
	Py_DECREF(seq);
	return false;
      }
    }
    Py_DECREF(seq);
    
    size_t checklen = len;
    int const status = m_hal->speed_set(qd, &checklen);
    if (0 != status) {
      cerr << "npm::PyBot::PrepareAction(): HAL speed_set failed (error code " << status << ".\n";
      return false;
    }
    if (checklen != len) {
      cerr << "npm::PyBot::PrepareAction(): expected " << checklen
	   << " values, but Python update function provided " << len << ".\n";
      return false;
    }
    
    return true;
  }
  
  
  void PyBot::
  SetGoal(double timestep, const Goal & goal)
  {
    m_goal = goal;
  }
  
  
  bool PyBot::
  GetGoal(Goal &goal)
  {
    goal = m_goal;
    return true;
  }
  
  
  bool PyBot::
  GoalReached()
  {
    return m_goal.DistanceReached(m_server->GetTruePose());
  }
  
  
  bool PyBotCreator::
  init(std::string const &module,
       std::string const &createfunc)
  {
    return false;
  }
  
  
  PyBot * PyBotCreator::
  create(string const & instance_name)
  {
    
    if (!PyMapping_Check(res)) {
      cerr << "npm::PyBot::PrepareAction(): Python update function did not return a mapping\n";
      Py_DECREF(res);
      return false;
    }
  }
  
}
