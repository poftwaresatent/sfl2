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
#include <cmath>
#include <algorithm>
#include <iostream>


using namespace std;


namespace sfl {


  DynamicWindow::
  DynamicWindow(int dimension,
		double grid_width,
		double grid_height,
		double grid_resolution,
		const RobotModel & robot_model,
		const MotionController & motion_controller,
		double alpha_distance,
		double alpha_heading,
		double alpha_speed):
    _reachableQd(robot_model.Timestep() * robot_model.QddMax()),
    _dimension(dimension),
    _resolution(2 * robot_model.QdMax() / dimension),
    _alphaDistance(alpha_distance),
    _alphaHeading(alpha_heading),
    _alphaSpeed(alpha_speed),
    _robot_model(robot_model),
    _motion_controller(motion_controller),
    _distance_objective(*this,
			robot_model,
			grid_width,
			grid_height,
			grid_resolution),
    _heading_objective(*this, robot_model),
    _speed_objective(*this, robot_model),
    _qdlOpt(-1),
    _qdrOpt(-1)
  {
    // allocations
    _qd = new double[_dimension];

    _stopDistance = new double[_dimension];

    _state = new speedstate_t*[_dimension];
    for(int i = 0; i < _dimension; ++i)
      _state[i] = new speedstate_t[_dimension];

    _objective = new double*[_dimension];
    for(int i = 0; i < _dimension; ++i)
      _objective[i] = new double[_dimension];

    // initialisations
    for(int i = 0; i < _dimension; ++i)
      _qd[i] = FindQd(i);
  
    for(int i = 0; i < _dimension; ++i)
      _stopDistance[i] =
	0.5 * _robot_model.WheelRadius()
	* _qd[i] * _qd[i]
	/ _robot_model.QddMax();

    InitForbidden();

    _distance_objective.Initialize(& cerr);
    _heading_objective.Initialize(0);
    _speed_objective.Initialize(0);
  }


  DynamicWindow::
  ~DynamicWindow()
  {
    delete[] _qd;
    delete[] _stopDistance;

    for(int i = 0; i < _dimension; ++i)
      delete[] _state[i];
    delete[] _state;

    for(int i = 0; i < _dimension; ++i)
      delete[] _objective[i];
    delete[] _objective;
  }


  bool DynamicWindow::
  Forbidden(int qdlIndex,
	    int qdrIndex)
    const
  {
    return _state[qdlIndex][qdrIndex] == FORBIDDEN;
  }


  bool DynamicWindow::
  Admissible(int qdlIndex,
	     int qdrIndex)
    const
  {
    return _state[qdlIndex][qdrIndex] == ADMISSIBLE;
  }


  bool DynamicWindow::
  Reachable(int qdlIndex,
	    int qdrIndex)
    const
  {
    return _state[qdlIndex][qdrIndex] == REACHABLE;
  }


  double DynamicWindow::
  Qd(int index)
    const
  {
    return _qd[index];
  }


  void DynamicWindow::
  Update(double dx,
	 double dy,
	 boost::shared_ptr<const Scan> local_scan,
	 ostream * dbgos)
  {
    double qdl, qdr;
    _motion_controller.GetActuators(qdl, qdr);
 
    CalculateReachable(qdl, qdr);

    _distance_objective.Calculate(_qdlMin, _qdlMax, _qdrMin, _qdrMax,
				  local_scan);

    CalculateAdmissible();

    _heading_objective.SetGoal(dx, dy);
    _heading_objective.Calculate(_qdlMin, _qdlMax, _qdrMin, _qdrMax);
    _speed_objective.Calculate(_qdlMin, _qdlMax, _qdrMin, _qdrMax);

    CalculateOptimum(_alphaDistance,
		     _alphaHeading,
		     _alphaSpeed);

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
	     double & local_y)
    const
  {
    _heading_objective.GetGoal(local_x, local_y);
  }


  void DynamicWindow::
  SetHeadingOffset(double angle)
  {
    _heading_objective.SetOffset(angle);
  }


  double DynamicWindow::
  GetHeadingOffset()
    const
  {
    return _heading_objective.GetOffset();
  }


  void DynamicWindow::
  GoFast()
  {
    _speed_objective.GoFast();
  }


  void DynamicWindow::
  GoSlow()
  {
    _speed_objective.GoSlow();
  }


  void DynamicWindow::
  GoForward()
  {
    _heading_objective.SetOffset(0);
    _speed_objective.GoForward();
  }


  void DynamicWindow::
  GoBackward()
  {
    _heading_objective.SetOffset(M_PI);
    _speed_objective.GoBackward();
  }


  bool DynamicWindow::
  OptimalActuators(double & qdl,
		   double & qdr)
    const
  {
    if((_qdlOpt < 0) || (_qdrOpt < 0))
      return false;

    qdl = _qd[_qdlOpt];
    qdr = _qd[_qdrOpt];

    return true;
  }


  int DynamicWindow::
  FindIndex(double qd)
    const
  {
    return (int) floor(0.5 * ((double) _dimension) + qd / _resolution);
  }


  double DynamicWindow::
  FindQd(int index)
    const
  {
    return _resolution * (((double) index) - 0.5 * ((double) _dimension - 1));
  }


  void DynamicWindow::
  InitForbidden()
  {
    for(int il = 0; il < _dimension; ++il)
      for(int ir = 0; ir < _dimension; ++ir){
	double sd, thetad;
	_robot_model.Actuator2Global(_qd[il], _qd[ir], sd, thetad);

	if((absval(sd) > _robot_model.SdMax()) ||
	   (absval(thetad) > _robot_model.ThetadMax()))
	  _state[il][ir] = FORBIDDEN;
      }
  }


  void DynamicWindow::
  CalculateReachable(double qdl,
		     double qdr)
  {
    _qdlMin = maxval(0             , FindIndex(qdl - _reachableQd));
    _qdlMax = minval(_dimension - 1, FindIndex(qdl + _reachableQd));
    _qdrMin = maxval(0             , FindIndex(qdr - _reachableQd));
    _qdrMax = minval(_dimension - 1, FindIndex(qdr + _reachableQd));
  
    for(int il = _qdlMin; il <= _qdlMax; ++il)
      for(int ir = _qdrMin; ir <= _qdrMax; ++ir)
	if(_state[il][ir] != FORBIDDEN)
	  _state[il][ir] = REACHABLE;  
  }


  void DynamicWindow::
  CalculateAdmissible()
  {
    double thresh(_distance_objective.MinValue() + epsilon);
  
    for(int il = _qdlMin; il <= _qdlMax; ++il)
      for(int ir = _qdrMin; ir <= _qdrMax; ++ir)
	if((_state[il][ir] != FORBIDDEN) &&
	   (_distance_objective.Value(il, ir) > thresh))
	  _state[il][ir] = ADMISSIBLE;
  }


  void DynamicWindow::
  CalculateOptimum(double alphaDistance,
		   double alphaHeading,
		   double alphaSpeed)
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
  }


  int DynamicWindow::
  Dimension()
    const
  {
    return _dimension;
  }


  void DynamicWindow::
  DumpObstacles(ostream & os,
		const char * prefix)
    const
  {
    _distance_objective.DumpGrid(os, prefix);
  }


  void DynamicWindow::
  DumpObjectives(ostream & os,
		 const char * prefix)
    const
  {
    for(int iqdr(_dimension - 1); iqdr > _qdrMax; --iqdr){
      os << prefix;
      for(int iqdl(0); iqdl < _dimension; ++iqdl)
	if(_state[iqdl][iqdr] == FORBIDDEN)
	  os << "#";
	else
	  os << "*";
      os << endl;
    }

    for(int iqdr(_qdrMax); iqdr >= _qdrMin; --iqdr){
      os << prefix;

      for(int iqdl(0); iqdl < _qdlMin; ++iqdl)
	if(_state[iqdl][iqdr] == FORBIDDEN)
	  os << "#";
	else
	  os << "*";

      for(int iqdl(_qdlMin); iqdl <= _qdlMax; ++iqdl){
	if((iqdl == _qdlOpt) && (iqdr == _qdrOpt)){
	  os << ":";
	  continue;
	}
	if(_state[iqdl][iqdr] == FORBIDDEN)
	  os << "#";
	else if(_state[iqdl][iqdr] == ADMISSIBLE)
	  os << ".";
	else
	  os << "x";
      }

      for(int iqdl(_qdlMax + 1); iqdl < _dimension; ++iqdl)
	if(_state[iqdl][iqdr] == FORBIDDEN)
	  os << "#";
	else
	  os << "*";

      os << endl;
    }
    
    for(int iqdr(_qdrMin - 1); iqdr >= 0; --iqdr){
      os << prefix;
      for(int iqdl(0); iqdl < _dimension; ++iqdl)
	if(_state[iqdl][iqdr] == FORBIDDEN)
	  os << "#";
	else
	  os << "*";
      os << endl;
    }
  }
  
}
