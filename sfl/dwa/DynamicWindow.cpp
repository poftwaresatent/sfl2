/* 
 * Copyright (C) 2004
 * Swiss Federal Institute of Technology, Lausanne. All rights reserved.
 * 
 * Developed at the Autonomous Systems Lab.
 * Visit our homepage at http://asl.epfl.ch/
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


#include "DynamicWindow.hpp"
#include <sfl/util/pdebug.hpp>
#include <cmath>
#include <algorithm>
#include <iostream>


#ifdef DEBUG
# define PDEBUG PDEBUG_ERR
# define PVDEBUG PDEBUG_OFF
#else // ! DEBUG
# define PDEBUG PDEBUG_OFF
# define PVDEBUG PDEBUG_OFF
#endif // DEBUG


using namespace boost;
using namespace std;


namespace sfl {


  DynamicWindow::
  DynamicWindow(int dimension,
		double grid_width,
		double grid_height,
		double grid_resolution,
		shared_ptr<const RobotModel> robot_model,
		const MotionController & motion_controller,
		double _alpha_distance,
		double _alpha_heading,
		double _alpha_speed,
		bool auto_init)
    : alpha_distance(_alpha_distance),
      alpha_heading(_alpha_heading),
      alpha_speed(_alpha_speed),
      _dimension(dimension),
      _maxindex(dimension - 1),
      _resolution(2 * robot_model->QdMax() / dimension),
      m_robot_model(robot_model),
      _motion_controller(motion_controller),
      _distance_objective(*this,
			  robot_model,
			  grid_width,
			  grid_height,
			  grid_resolution),
      _heading_objective(*this, *robot_model),
      _speed_objective(*this, *robot_model),
      _qdlMin(-1),
      _qdlMax(-1),
      _qdrMin(-1),
      _qdrMax(-1),
      _qdlOpt(-1),
      _qdrOpt(-1),
      _qddMax(robot_model->QddMax()),
      m_compute_next_optimum(false)
  {
    // allocations
    _qd = new double[_dimension];

    _state = new speedstate_t*[_dimension];
    for(int i = 0; i < _dimension; ++i)
      _state[i] = new speedstate_t[_dimension];

    _objective = new double*[_dimension];
    for(int i = 0; i < _dimension; ++i)
      _objective[i] = new double[_dimension];

    if(auto_init)
      Initialize(& cerr, false);
  }
  
  
  bool DynamicWindow::
  Initialize(ostream * os,
	     bool paranoid)
  {
    for(int i = 0; i < _dimension; ++i)
      _qd[i] = FindQd(i);
    
    InitForbidden();
    
    _distance_objective.Initialize(os);
    
    if(paranoid){
      _heading_objective.Initialize(os);
      _speed_objective.Initialize(os);
      if( ! _distance_objective.CheckLookup(os))
	return false;
    }
    else{
      _heading_objective.Initialize(0);
      _speed_objective.Initialize(0);
    }
    return true;
  }
  
  
  DynamicWindow::
  ~DynamicWindow()
  {
    delete[] _qd;

    for(int i = 0; i < _dimension; ++i)
      delete[] _state[i];
    delete[] _state;

    for(int i = 0; i < _dimension; ++i)
      delete[] _objective[i];
    delete[] _objective;
  }


  bool DynamicWindow::
  Forbidden(int qdlIndex,
	    int qdrIndex) const
  {
    return _state[qdlIndex][qdrIndex] == FORBIDDEN;
  }


  bool DynamicWindow::
  Admissible(int qdlIndex,
	     int qdrIndex) const
  {
    return _state[qdlIndex][qdrIndex] == ADMISSIBLE;
  }


  bool DynamicWindow::
  Reachable(int qdlIndex,
	    int qdrIndex) const
  {
    return _state[qdlIndex][qdrIndex] == REACHABLE;
  }


  double DynamicWindow::
  Qd(int index) const
  {
    return _qd[index];
  }


  void DynamicWindow::
  Update(double timestep, double dx, double dy,
	 boost::shared_ptr<const Scan> local_scan,
	 ostream * dbgos)
  {
    double qdl, qdr;
    _motion_controller.GetCurrentAct(qdl, qdr);
    PDEBUG("DWA dt: %g   goal: %g   %g   qd: %g   %g\n",
	   timestep, dx, dy, qdl, qdr);
    
    CalculateReachable(timestep, qdl, qdr);
    _distance_objective.Calculate(timestep, _qdlMin, _qdlMax, _qdrMin, _qdrMax,
				  local_scan);
    CalculateAdmissible();
    
    _heading_objective.local_goal_x = dx;
    _heading_objective.local_goal_y = dy;
    _heading_objective.Calculate(timestep, _qdlMin, _qdlMax, _qdrMin, _qdrMax);
    _speed_objective.Calculate(_qdlMin, _qdlMax, _qdrMin, _qdrMax);
    
    m_compute_next_optimum = true;
    
    if(dbgos != 0){
      (*dbgos) << "INFO from DynamicWindow::Update():\n"
	       << "  obstacles:\n";
      DumpObstacles((*dbgos), "    ");
      (*dbgos) << "  dynamic window:\n";
      DumpObjectives((*dbgos), "    ");
      (*dbgos) << "  FINISHED DynamicWindow::Update():\n";
    }
  }
  
  
  void DynamicWindow::
  GetSubGoal(double & local_x,
	     double & local_y) const
  {
    local_x = _heading_objective.local_goal_x;
    local_y = _heading_objective.local_goal_y;
  }


  void DynamicWindow::
  SetHeadingOffset(double angle)
  {
    _heading_objective.angle_offset = angle;
  }


  double DynamicWindow::
  GetHeadingOffset() const
  {
    return _heading_objective.angle_offset;
  }
  
  
  void DynamicWindow::
  GoFast()
  {
    _speed_objective.GoFast();
  }
  
  
  void DynamicWindow::
  GoStrictFast()
  {
    _speed_objective.GoStrictFast();
  }


  void DynamicWindow::
  GoSlow()
  {
    _speed_objective.GoSlow();
  }


  void DynamicWindow::
  GoStrictSlow()
  {
    _speed_objective.GoStrictSlow();
  }


  void DynamicWindow::
  GoForward()
  {
    _heading_objective.angle_offset = 0;
    _speed_objective.GoForward();
  }


  void DynamicWindow::
  GoBackward()
  {
    _heading_objective.angle_offset = M_PI;
    _speed_objective.GoBackward();
  }


  bool DynamicWindow::
  OptimalActuators(double & qdl, double & qdr) const
  {
    if(m_compute_next_optimum){
      CalculateOptimum(alpha_distance, alpha_heading, alpha_speed);
      m_compute_next_optimum = false;
    }
    if((_qdlOpt < 0) || (_qdrOpt < 0))
      return false;
    qdl = _qd[_qdlOpt];
    qdr = _qd[_qdrOpt];
    return true;
  }


  int DynamicWindow::
  FindIndex(double qd) const
  {
    return (int) floor(0.5 * ((double) _dimension) + qd / _resolution);
  }


  double DynamicWindow::
  FindQd(int index) const
  {
    return _resolution * (((double) index) - 0.5 * ((double) _dimension - 1));
  }


  void DynamicWindow::
  InitForbidden()
  {
    const double sd_max(m_robot_model->SdMax());
    const double thetad_max(m_robot_model->ThetadMax());
    for(int il = 0; il < _dimension; ++il)
      for(int ir = 0; ir < _dimension; ++ir){
	double sd, thetad;
	m_robot_model->Actuator2Global(_qd[il], _qd[ir], sd, thetad);
	if((absval(sd) > sd_max) || (absval(thetad) > thetad_max))
	  _state[il][ir] = FORBIDDEN;
      }
  }


  void DynamicWindow::
  CalculateReachable(double timestep, double qdl, double qdr)
  {
    const double reachableQd(timestep * _qddMax);
    _qdlMin = boundval(0, FindIndex(qdl - reachableQd), _maxindex);
    _qdlMax = boundval(0, FindIndex(qdl + reachableQd), _maxindex);
    _qdrMin = boundval(0, FindIndex(qdr - reachableQd), _maxindex);
    _qdrMax = boundval(0, FindIndex(qdr + reachableQd), _maxindex);
    for(int il = _qdlMin; il <= _qdlMax; ++il)
      for(int ir = _qdrMin; ir <= _qdrMax; ++ir)
	if(_state[il][ir] != FORBIDDEN)
	  _state[il][ir] = REACHABLE;  
  }


  void DynamicWindow::
  CalculateAdmissible()
  {
    const double thresh(_distance_objective.minValue + epsilon);
    for(int il = _qdlMin; il <= _qdlMax; ++il)
      for(int ir = _qdrMin; ir <= _qdrMax; ++ir)
	if((_state[il][ir] != FORBIDDEN) &&
	   (_distance_objective.Value(il, ir) > thresh))
	  _state[il][ir] = ADMISSIBLE;
  }


  void DynamicWindow::
  CalculateOptimum(double alphaDistance,
		   double alphaHeading,
		   double alphaSpeed) const
  {
    _qdlOpt = -1;
    _qdrOpt = -1;
    _objectiveMax = 0;
    _objectiveMin = alphaDistance + alphaHeading + alphaSpeed;

    for(int il = _qdlMin; il <= _qdlMax; ++il)
      for(int ir = _qdrMin; ir <= _qdrMax; ++ir)
	if(_state[il][ir] == ADMISSIBLE){
	  _objective[il][ir] =
	    alphaDistance * _distance_objective.Value(il, ir)
	    + alphaHeading  * _heading_objective.Value(il, ir)
	    + alphaSpeed    * _speed_objective.Value(il, ir);
	
	  if(_objective[il][ir] > _objectiveMax){
	    _objectiveMax = _objective[il][ir];
	    _qdlOpt = il;
	    _qdrOpt = ir;
	  }
	  if(_objective[il][ir] < _objectiveMin)
	    _objectiveMin = _objective[il][ir];
	}
    PDEBUG("DWA [%d   %d]: %g\n", _qdlOpt, _qdrOpt, _objectiveMax);
  }


  int DynamicWindow::
  Dimension() const
  {
    return _dimension;
  }


  void DynamicWindow::
  DumpObstacles(ostream & os,
		const char * prefix) const
  {
    _distance_objective.DumpGrid(os, prefix);
  }


  void DynamicWindow::
  DumpObjectives(ostream & os,
		 const char * prefix) const
  {
    static const char forbidden_char('.');
    static const char nonforbidden_char(' ');
    static const char optimum_char('O');
    static const char admissible_char(':');
    static const char collision_char('*');
    
    for(int iqdr(_dimension - 1); iqdr > _qdrMax; --iqdr){
      os << prefix;
      for(int iqdl(0); iqdl < _dimension; ++iqdl)
	if(_state[iqdl][iqdr] == FORBIDDEN) os << forbidden_char;
	else                                os << nonforbidden_char;
      os << "\n";
    }
    for(int iqdr(_qdrMax); iqdr >= _qdrMin; --iqdr){
      os << prefix;
      for(int iqdl(0); iqdl < _qdlMin; ++iqdl)
	if(_state[iqdl][iqdr] == FORBIDDEN) os << forbidden_char;
	else                                os << nonforbidden_char;
      for(int iqdl(_qdlMin); iqdl <= _qdlMax; ++iqdl){
	if((iqdl == _qdlOpt) && (iqdr == _qdrOpt)) os << optimum_char;
	else if(_state[iqdl][iqdr] == FORBIDDEN)   os << forbidden_char;
	else if(_state[iqdl][iqdr] == ADMISSIBLE)  os << admissible_char;
	else                                       os << collision_char;
      }
      for(int iqdl(_qdlMax + 1); iqdl < _dimension; ++iqdl)
	if(_state[iqdl][iqdr] == FORBIDDEN) os << forbidden_char;
	else                                os << nonforbidden_char;
      os << "\n";
    }
    for(int iqdr(_qdrMin - 1); iqdr >= 0; --iqdr){
      os << prefix;
      for(int iqdl(0); iqdl < _dimension; ++iqdl)
	if(_state[iqdl][iqdr] == FORBIDDEN) os << forbidden_char;
	else                                os << nonforbidden_char;
      os << "\n";
    }
  }
  
}
