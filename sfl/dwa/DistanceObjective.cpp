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


#include "DistanceObjective.hpp"
#include "DynamicWindow.hpp"
#include <sfl/util/Ray.hpp>
#include <cmath>
#include <iostream>


using boost::shared_ptr;
using boost::scoped_array;
using boost::scoped_ptr;
using std::ostream;
using std::vector;
using std::cerr;		// dbg


namespace sfl {
  
  
  DistanceObjective::
  DistanceObjective(const DynamicWindow & dynamic_window,
		    const RobotModel & robot_model,
		    double grid_width,
		    double grid_height,
		    double grid_resolution):
    Objective(dynamic_window),
    _securityDistance(robot_model.SecurityDistance()),
    _maxTime(robot_model.QdMax() / robot_model.QddMax()),
    _gridResolution(grid_resolution),
    _robot_model(robot_model),
    _hull(robot_model.GetHull()),
    _x0( - grid_width  / 2),
    _y0( - grid_height / 2),
    _x1(   grid_width  / 2),
    _y1(   grid_height / 2)
  {
    _qdLookup.reset(new double[_dimension]);

    _maxTimeLookup.reset(new scoped_array<double>[_dimension]);
    for(unsigned int i = 0; i < _dimension; ++i)
      _maxTimeLookup[i].reset(new double[_dimension]);

    // could be a little more paranoid and absval() everything...

    double delta(_x1 - _x0);
    double dim(ceil(delta / _gridResolution));
    _dx = delta / dim;
    _dxInv = dim / delta;
    _dimx = (int) dim;

    delta = _y1 - _y0;
    dim = ceil(delta / _gridResolution);
    _dy = delta / dim;
    _dyInv = dim / delta;
    _dimy = (int) dim;

    // allocate _grid[][]
    _grid.reset(new scoped_array<bool>[_dimx]);
    for(int i = 0; i < _dimx; ++i)
      _grid[i].reset(new bool[_dimy]);
    
    // allocate timeLookup[][]
    _timeLookup.reset(new scoped_array<scoped_ptr<Lookup> >[_dimx]);
    for(int i(0); i < _dimx; ++i){
      // NOTE: scoped_array constructs to zero, which we rely on!
      _timeLookup[i].reset(new scoped_ptr<Lookup>[_dimy]);
    }
    
    // determine padded hull
    double padding(sqrt(_dx * _dx + _dy * _dy));
    _paddedHull = _hull->CreateGrownHull(padding);

    for(int i(0); i < _paddedHull->GetNPoints(); ++i)
      _outline.push_back(_paddedHull->GetLine(i));
    
    // some more inter-version compatibility
    Polygon foo;
    foo.AddPoint(_x0, _y0);
    foo.AddPoint(_x1, _y0);
    foo.AddPoint(_x1, _y1);
    foo.AddPoint(_x0, _y1);
    _evaluationHull = foo.CreateConvexHull();
  }
  
  
  void DistanceObjective::
  Initialize(ostream * progress_stream)
  {
    // precalculate lookup tables
    for(unsigned int i = 0; i < _dimension; ++i)
      _qdLookup[i] = _dynamic_window.Qd(i);

    for(unsigned int i = 0; i < _dimension; ++i)
      for(unsigned int j = 0; j < _dimension; ++j)
	_maxTimeLookup[i][j] =
	  maxval(absval(_qdLookup[i]), absval(_qdLookup[j])) /
	  _robot_model.QddMax();

    if(progress_stream != 0)
      (*progress_stream) << "DistanceObjective::Initialize()\n"
			 << "   calculating main lookup table...\n";
    
    for(int igy = _dimy - 1; igy >= 0; --igy){
      double y(FindYlength(igy));

      for(int igx = 0; igx < _dimx; ++igx){
	double x(FindXlength(igx));
      
	if(_evaluationHull->Contains(x, y) &&
	   ! _paddedHull->Contains(x, y)){
	  unsigned int nValidCollisions(0);

	  // fill the actuator lookup table with collision times
	  for(unsigned int iqdl = 0; iqdl < _dimension; ++iqdl)
	    for(unsigned int iqdr = 0; iqdr < _dimension; ++iqdr)
	      if(_dynamic_window.Forbidden(iqdl, iqdr))
		Lookup::LoadBuffer(iqdl, iqdr, - 1);
	      else{
		double t(PredictCollision(_qdLookup[iqdl], _qdLookup[iqdr],
					  x, y));

		if((t > 0) && (t <= _maxTime)){
		  ++nValidCollisions;
		  Lookup::LoadBuffer(iqdl, iqdr, t);
		}
		else
		  Lookup::LoadBuffer(iqdl, iqdr, - 1);
	      }
	
	  if(nValidCollisions > 0){
	    if(progress_stream != 0)
	      (*progress_stream) << "*";

	    // allocate a lookup table for this grid cell
	    _timeLookup[igx][igy].reset(new Lookup(_dimension, 0, _maxTime));
	
	    // tell timeLookup to store the distances (compressed)
	    _timeLookup[igx][igy]->SaveBuffer();
	  }
	  else
	    if(progress_stream != 0)
	      (*progress_stream) << "o";
	}
	else{
	  // fill it with epsilon collision time to make the robot stop if
	  // there's something inside the outline (which is quite a hack
	  // because the same could be achieved using a simple flag,
	  // which wastes less ram and time... ah well)
	  //
	  // can't simply use -1 because then the compression code
	  // segfaults (it's looking for a minimum over positive
	  // values), can't simply use 0 because somewhere else in this
	  // code here I chack against zero... in short,
	  // DistanceObjective NEEDS A GENERAL OVERHAUL!!!
	
	  for(unsigned int iqdl = 0; iqdl < _dimension; ++iqdl)
	    for(unsigned int iqdr = 0; iqdr < _dimension; ++iqdr)
	      Lookup::LoadBuffer(iqdl, iqdr, epsilon);
	  _timeLookup[igx][igy].reset(new Lookup(_dimension, 0, _maxTime));
	  _timeLookup[igx][igy]->SaveBuffer();
	
	  if(progress_stream != 0)
	    (*progress_stream) << ".";
	}
      }
      if(progress_stream != 0)
	(*progress_stream) << "\n";
    }
    if(progress_stream != 0)
      (*progress_stream) << "   finished.\n";
  }
  
  
  bool DistanceObjective::
  CheckLookup(ostream & os)
    const
  {
    os << "INFO from DistanceObjective::CheckLookup():\n";
    for(unsigned int i = 0; i < _dimension; ++i)
      if(_qdLookup[i] != _dynamic_window.Qd(i)){
	os << "  ERROR _qdLookup[" << i << "] is wrong\n";
	return false;
      }
    
    for(unsigned int i = 0; i < _dimension; ++i){
      for(unsigned int j = 0; j < _dimension; ++j)
	if(_maxTimeLookup[i][j]
	   !=
	   maxval(absval(_qdLookup[i]),
		  absval(_qdLookup[j])) /
	   _robot_model.QddMax()){
	  os << "  ERROR _maxTimeLookup["<<i<<"]["<<j<<"] is wrong\n";
	  return false;
	}
    }
    
    for(int igy = _dimy - 1; igy >= 0; --igy){
      os << "  ";
      double y(FindYlength(igy));
      for(int igx = 0; igx < _dimx; ++igx){
	double x(FindXlength(igx));
	for(unsigned int iqdl = 0; iqdl < _dimension; ++iqdl){
	  for(unsigned int iqdr = 0; iqdr < _dimension; ++iqdr){
	    double wanted(-1);
	    if(_evaluationHull->Contains(x, y))
	      if(_paddedHull->Contains(x, y))
		wanted = epsilon; // due to epsilon hack above...
	      else{
		if( ! _dynamic_window.Forbidden(iqdl, iqdr)){
		  const double t(PredictCollision(_qdLookup[iqdl],
						  _qdLookup[iqdr],
						  x, y));
		  if((t > 0) && (t <= _maxTime))
		    wanted = t;
		}
	      }
	    double compressed(-1);
	    if(_timeLookup[igx][igy])
	      compressed = _timeLookup[igx][igy]->Get(iqdl, iqdr);
	    
	    if(wanted != compressed){
	      if(0 > wanted){
		if(_timeLookup[igx][igy]){
		  os << "\n  ERROR _timeLookup[" << igx << "][" << igy
		     << "] should be -1 but is " << compressed << "\n"
		     << "  cell [" << igx << "][" << igy << "] at (" << x
		     << ", " << y << ")\n";
		  return false;
		}
		os << "\nBUG in check procedure?\n";
		return false;
	      }
	      if(0 > compressed){
		os << "\n  ERROR _timeLookup[" << igx << "][" << igy
		   << "] should be " << wanted << " but is "
		   << compressed << " (which should be -1)\n"
		   << "  cell [" << igx << "][" << igy << "] at (" << x
		   << ", " << y << ")\n";
		return false;
	      }
	    }
	    // update stats here?
	  }
	}
	os << ".";
      }
      os << "\n";
    }
    return true;
  }
  
  
  void DistanceObjective::
  Calculate(unsigned int qdlMin,
	    unsigned int qdlMax,
	    unsigned int qdrMin,
	    unsigned int qdrMax,
	    boost::shared_ptr<const Scan> local_scan)
  {
    ResetGrid();
    UpdateGrid(local_scan);

    for(unsigned int iqdl = qdlMin; iqdl <= qdlMax; ++iqdl)
      for(unsigned int iqdr = qdrMin; iqdr <= qdrMax; ++iqdr)      
	if( ! _dynamic_window.Forbidden(iqdl, iqdr)){
	  double t(MinTime(iqdl, iqdr));

	  _value[iqdl][iqdr] =
	    CalculateValue(t, _maxTimeLookup[iqdl][iqdr]);
	}
  }


  void DistanceObjective::
  ResetGrid()
  {
    for(int ix = 0; ix < _dimx; ++ix)
      for(int iy = 0; iy < _dimy; ++iy)
	_grid[ix][iy] = false;
  }


  void DistanceObjective::
  UpdateGrid(boost::shared_ptr<const Scan> local_scan)
  {
    const Scan::array_t & ldata(local_scan->data);
    const size_t nscans(ldata.size());
    for(size_t is(0); is < nscans; ++is)
      if((ldata[is].locx >= _x0) &&
	 (ldata[is].locx <= _x1) && 
	 (ldata[is].locy >= _y0) &&
	 (ldata[is].locy <= _y1))
	_grid[FindXindex(ldata[is].locx)][FindYindex(ldata[is].locy)] = true;
    
    static const bool dump_grid(true);
    if(dump_grid){
      cerr << "DEBUG DistanceObjective::UpdateGrid():\n";
      DumpGrid(cerr, "  ");
    }
  }
  
  
  double DistanceObjective::
  MinTime(unsigned int iqdl,
	  unsigned int iqdr)
  {
    // easy speedup: remember how many points there are, return as soon as
    //    all are processed.
    double minTime(_maxTime);
  
    for(int ix = 0; ix < _dimx; ++ix)
      for(int iy = 0; iy < _dimy; ++iy)
	if(_grid[ix][iy] &&	// cell is occupied
	   (_timeLookup[ix][iy])){ // cell is inside evaluation region
	  double t(_timeLookup[ix][iy]->Get(iqdl, iqdr));
	
	  if((t > 0) && (t < minTime))
	    minTime = t;
	}

    return minTime;
  }


  int DistanceObjective::
  FindXindex(double d)
    const
  {
    return (int) floor((d - _x0) * _dxInv);
  }


  double DistanceObjective::
  FindXlength(int i)
    const
  {
    return (0.5 + (double) i) * _dx + _x0;
  }


  int DistanceObjective::
  FindYindex(double d)
    const
  {
    return (int) floor((d - _y0) * _dyInv);
  }


  double DistanceObjective::
  FindYlength(int i)
    const
  {
    return (0.5 + (double) i) * _dy + _y0;
  }


  double DistanceObjective::
  PredictCollision(double qdl,
		   double qdr,
		   double lx,
		   double ly)
    const
  {
    double tMin(-1);

    double sd, thetad;
    _robot_model.Actuator2Global(qdl, qdr, sd, thetad);

    if(absval(epsilon * sd) < absval(thetad / epsilon)){
      // circular equation
      double thetadInv(absval(1 / thetad));

      double rCur(sd / thetad);
      double rGir(      lx      *     lx
			+ (ly - rCur) * (ly - rCur));
      rGir = sqrt(rGir);
      double phi0(atan2((ly - rCur), lx));

      // loop over outline
      for(vector<shared_ptr<Line> >::const_iterator iline(_outline.begin());
	  iline != _outline.end();
	  ++iline){
	double qx[2], qy[2];

	bool v[2];
	(*iline)->CircleIntersect(0, rCur, rGir,
				  qx[0], qy[0],
				  qx[1], qy[1],
				  v[0], v[1]);

	for(unsigned int i = 0; i < 2; ++i)
	  if(v[i]){
	    double phi(atan2(qy[i] - rCur, qx[i]) - phi0);
	    if(thetad >= 0)
	      phi = mod2pi( - phi);
	    else
	      phi = mod2pi(   phi);
	
	    double t(phi * thetadInv);
	  
	    if((tMin < 0) || (t < tMin)){
	      tMin = t;
	    }
	  }
      } // end loop over local lines
    }
    else if(absval(sd) > epsilon){
      // straight line approximation
      double sdInv(absval(1 / sd));

      Ray *ray;
      if(sd >= 0)
	ray = new Ray(Frame(lx, ly, M_PI));	// hax: port to npm
      else
	ray = new Ray(Frame(lx, ly, 0));	// hax: port to npm
    
      // loop over local lines
      for(vector<shared_ptr<Line> >::const_iterator iline(_outline.begin());
	  iline != _outline.end(); ++iline){
	// calculate distance
	double t(sdInv * ray->Intersect(**iline));
      
	// check for new minimum
	if((t >= 0) &&
	   ((tMin < 0) || (t < tMin))){
	  tMin = t;
	}
      }

      delete ray;
    }
    // else don't touch tMin, -1 means "no collision"

    return tMin;
  }


  double DistanceObjective::
  CalculateValue(double measure,
		 double floor)
  {
    if(measure > _maxTime)
      return _maxValue;
  
    if(measure <= floor)
      return _minValue;

    return
      _minValue +
      (measure - floor) *
      (_maxValue - _minValue) /
      (_maxTime - floor);
  }


  void DistanceObjective::
  GetRange(double & x0,
	   double & y0,
	   double & x1,
	   double & y1)
    const
  {
    x0 = _x0;
    y0 = _y0;
    x1 = _x1;
    y1 = _y1;
  }

  
  void DistanceObjective::
  DumpGrid(std::ostream & os, const char * prefix)
    const
  {
    for(int igy = _dimy - 1; igy >= 0; --igy){
      double y(FindYlength(igy));
      os << prefix;
      for(int igx = 0; igx < _dimx; ++igx){
	double x(FindXlength(igx));
	if(_paddedHull->Contains(x, y)){
	  if(_grid[igx][igy])
	    os << "o";
	  else
	    os << ".";
	}
	else{
	  if(_grid[igx][igy]){
	    if( ! _timeLookup[igx][igy])
	      os << "o";
	    else
	      os << "x";
	  }
	  else{
	    if( ! _timeLookup[igx][igy])
	      os << ".";
	    else
	      os << "-";
	  }
	}
      }
      os << "\n";
    }
  }
  
  
  double DistanceObjective::
  CollisionTime(int ix, int iy, int iqdl, int iqdr) const
  {
    if(_timeLookup[ix][iy])
      return _timeLookup[ix][iy]->Get(iqdl, iqdr);
    return -1;
  }
  
}
