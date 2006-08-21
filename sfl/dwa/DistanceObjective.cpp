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
#include "Lookup.hpp"
#include <sfl/util/Ray.hpp>
#include <cmath>
#include <iostream>


using boost::shared_ptr;
using std::ostream;
using std::vector;


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
    _y1(   grid_height / 2),
    _maxTimeLookup(dimension, dimension, invalidTime)
  {
    double delta(absval(_x1 - _x0));
    double dim(ceil(absval(delta / _gridResolution)));
    _dx = delta / dim;
    _dxInv = dim / delta;
    _dimx = static_cast<size_t>(dim);
    
    delta = absval(_y1 - _y0);
    dim = ceil(absval(delta / _gridResolution));
    _dy = delta / dim;
    _dyInv = dim / delta;
    _dimy = static_cast<size_t>(dim);
    
    _grid.reset(new array2d<bool>(_dimx, _dimy, false));
    _timeLookup.reset(new array2d<shared_ptr<Lookup> >(_dimx, _dimy));
    
    // determine padded hull
    double padding(sqrt(_dx * _dx + _dy * _dy));
    _paddedHull = _hull->CreateGrownHull(padding);

    for(size_t ii(0); ii < _paddedHull->GetNPoints(); ++ii)
      _outline.push_back(_paddedHull->GetLine(ii));
    
    // some more inter-version compatibility
    Polygon foo;
    foo.AddPoint(_x0, _y0);
    foo.AddPoint(_x1, _y0);
    foo.AddPoint(_x1, _y1);
    foo.AddPoint(_x0, _y1);
    _evaluationHull.reset(new Hull(foo));
  }
  
  
  void DistanceObjective::
  Initialize(ostream * progress_stream)
  {
    // precalculate lookup tables
    _qdLookup.clear();
    for(size_t ii(0); ii < dimension; ++ii)
      _qdLookup.push_back(m_dynamic_window.Qd(ii));
    
    for(size_t ii(0); ii < dimension; ++ii)
      for(size_t jj(0); jj < dimension; ++jj)
	_maxTimeLookup[ii][jj] =
	  maxval(absval(_qdLookup[ii]), absval(_qdLookup[jj])) /
	  _robot_model.QddMax();
    
    if(progress_stream != 0)
      (*progress_stream) << "DistanceObjective::Initialize()\n"
			 << "   calculating main lookup table...\n";
    
    for(ssize_t igy(_dimy - 1); igy >= 0; --igy){
      double yy(FindYlength(igy));
      
      for(size_t igx(0); igx < _dimx; ++igx){
	double xx(FindXlength(igx));
	
	if(_evaluationHull->Contains(xx, yy) &&
	   ! _paddedHull->Contains(xx, yy)){
	  array2d<double> buffer(dimension, dimension, invalidTime);
	  size_t nValidCollisions(0);
	  
	  // fill the actuator lookup table with collision times
	  for(size_t iqdl(0); iqdl < dimension; ++iqdl)
	    for(size_t iqdr(0); iqdr < dimension; ++iqdr)
	      if( ! m_dynamic_window.Forbidden(iqdl, iqdr)){
		double tt(PredictCollision(_qdLookup[iqdl], _qdLookup[iqdr],
					   xx, yy));
		if((tt > 0) && (tt <= _maxTime)){
		  ++nValidCollisions;
		  buffer[iqdl][iqdr] = tt;
		}
	      }
	  
	  if(nValidCollisions > 0){
	    if(progress_stream != 0)
	      (*progress_stream) << "*";
	    (*_timeLookup)[igx][igy].reset(new Lookup(buffer, 0, _maxTime,
						   invalidTime));
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
	  // can't simply use invalidTime because that's treated like
	  // "no collision", can't use 0 because somewhere else in
	  // this code here I check against zero... we NEED A GENERAL
	  // OVERHAUL HERE!!!
	  array2d<double> buffer(dimension, dimension, epsilon);
	  (*_timeLookup)[igx][igy].reset(new Lookup(buffer, 0, _maxTime,
						 invalidTime));
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
  CheckLookup(ostream * os)
    const
  {
    if(0 != os)
      (*os) << "INFO from DistanceObjective::CheckLookup():\n";
    for(size_t ii(0); ii < dimension; ++ii)
      if(_qdLookup[ii] != m_dynamic_window.Qd(ii)){
	if(0 != os)
	  (*os) << "  ERROR _qdLookup[" << ii << "] is " << _qdLookup[ii]
		<< " but should be " << m_dynamic_window.Qd(ii) << "\n";
	return false;
      }
    
    for(size_t ii(0); ii < dimension; ++ii)
      for(size_t jj(0); jj < dimension; ++jj){
	const double check(maxval(absval(_qdLookup[ii]), absval(_qdLookup[jj]))
			   / _robot_model.QddMax());
	if(epsilon < absval(_maxTimeLookup[ii][jj] - check)){
	  if(0 != os)
	    (*os) << "  ERROR _maxTimeLookup[" << ii << "][" << jj << "] is "
		  << _maxTimeLookup[ii][jj] << "but should be "
		  << check << "\n"
		  << "        difference = " << _maxTimeLookup[ii][jj] - check
		  << "\n";
	  return false;
	}
      }
    
    for(ssize_t igy(_dimy - 1); igy >= 0; --igy){
      if(0 != os)
	(*os) << "  ";
      double yy(FindYlength(igy));
      for(size_t igx(0); igx < _dimx; ++igx){
	double xx(FindXlength(igx));
	for(size_t iqdl(0); iqdl < dimension; ++iqdl){
	  for(size_t iqdr(0); iqdr < dimension; ++iqdr){
	    double wanted(invalidTime);
	    if(_evaluationHull->Contains(xx, yy))
	      if(_paddedHull->Contains(xx, yy))
		wanted = epsilon; // due to epsilon hack above...
	      else{
		if( ! m_dynamic_window.Forbidden(iqdl, iqdr)){
		  const double tt(PredictCollision(_qdLookup[iqdl],
						  _qdLookup[iqdr], xx, yy));
		  if((tt > 0) && (tt <= _maxTime))
		    wanted = tt;
		}
	      }
	    double compressed(invalidTime);
	    if((*_timeLookup)[igx][igy])
	      compressed = (*_timeLookup)[igx][igy]->Get(iqdl, iqdr);
	    
	    if(wanted != compressed){
	      if(invalidTime != wanted){
		if((*_timeLookup)[igx][igy]){
		  if(0 != os)
		    (*os) << "\n  ERROR (*_timeLookup)[" << igx << "][" << igy
			  << "] should be invalidTime but is " << compressed
			  << "\n  cell [" << igx << "][" << igy << "] at ("
			  << xx << ", " << yy << ")\n";
		  return false;
		}
		if(0 != os)
		  (*os) << "\nBUG in check procedure?\n";
		return false;
	      }
	      if(invalidTime != compressed){
		if(0 != os)
		  (*os) << "\n  ERROR (*_timeLookup)[" << igx << "][" << igy
			<< "] should be " << wanted << " but is "
			<< compressed << " (which should be invalidTime)\n"
			<< "  cell [" << igx << "][" << igy << "] at (" << xx
			<< ", " << yy << ")\n";
		return false;
	      }
	    }
	    // update stats here?
	  }
	}
	if(0 != os)
	  (*os) << ".";
      }
      if(0 != os)
	(*os) << "\n";
    }
    return true;
  }
  
  
  void DistanceObjective::
  Calculate(double timestep, size_t qdlMin, size_t qdlMax, size_t qdrMin,
	    size_t qdrMax, boost::shared_ptr<const Scan> local_scan)
  {
    ResetGrid();
    UpdateGrid(local_scan);
    for(size_t iqdl(qdlMin); iqdl <= qdlMax; ++iqdl)
      for(size_t iqdr(qdrMin); iqdr <= qdrMax; ++iqdr)      
	if( ! m_dynamic_window.Forbidden(iqdl, iqdr))
	  m_value[iqdl][iqdr] = CalculateValue(timestep + MinTime(iqdl, iqdr),
					       _maxTimeLookup[iqdl][iqdr]);
  }


  void DistanceObjective::
  ResetGrid()
  {
    for(size_t ix(0); ix < _dimx; ++ix)
      for(size_t iy(0); iy < _dimy; ++iy)
	(*_grid)[ix][iy] = false;
  }


  void DistanceObjective::
  UpdateGrid(boost::shared_ptr<const Scan> local_scan)
  {
    const size_t nscans(local_scan->GetNScans());
    for(size_t is(0); is < nscans; ++is){
      const Scan::data_t & ldata(local_scan->GetData(is));
      if((ldata.locx >= _x0) &&
	 (ldata.locx <= _x1) && 
	 (ldata.locy >= _y0) &&
	 (ldata.locy <= _y1))
	(*_grid)[FindXindex(ldata.locx)][FindYindex(ldata.locy)] = true;
    }
  }
  
  
  /** \todo easy speedup: remember how many points there are, return
      as soon as all are processed. */
  double DistanceObjective::
  MinTime(size_t iqdl,
	  size_t iqdr)
  {
    double minTime(_maxTime);
    for(size_t ix(0); ix < _dimx; ++ix)
      for(size_t iy(0); iy < _dimy; ++iy)
	if((*_grid)[ix][iy] &&	// cell is occupied
	   ((*_timeLookup)[ix][iy])){ // cell is inside evaluation region
	  double tt((*_timeLookup)[ix][iy]->Get(iqdl, iqdr));
	  if((tt != invalidTime) && (tt < minTime))
	    minTime = tt;
	}
    return minTime;
  }
  
  
  ssize_t DistanceObjective::
  FindXindex(double d)
    const
  {
    return static_cast<ssize_t>(floor((d - _x0) * _dxInv));
  }


  double DistanceObjective::
  FindXlength(ssize_t i)
    const
  {
    return (0.5 + (double) i) * _dx + _x0;
  }


  ssize_t DistanceObjective::
  FindYindex(double d)
    const
  {
    return static_cast<ssize_t>(floor((d - _y0) * _dyInv));
  }


  double DistanceObjective::
  FindYlength(ssize_t i)
    const
  {
    return (0.5 + (double) i) * _dy + _y0;
  }


  double DistanceObjective::
  PredictCollision(double qdl, double qdr, double lx, double ly) const
  {
    double tMin(invalidTime);
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
	bool vv[2];
	(*iline)->CircleIntersect(0, rCur, rGir,
				  qx[0], qy[0],
				  qx[1], qy[1],
				  vv[0], vv[1]);
	for(size_t ii(0); ii < 2; ++ii)
	  if(vv[ii]){
	    double phi(atan2(qy[ii] - rCur, qx[ii]) - phi0);
	    if(thetad >= 0)
	      phi = mod2pi( - phi);
	    else
	      phi = mod2pi(   phi);
	    const double tt(phi * thetadInv);
	    if((tMin < 0) || (tt < tMin))
	      tMin = tt;
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
      
      // loop over outline
      for(vector<shared_ptr<Line> >::const_iterator iline(_outline.begin());
	  iline != _outline.end(); ++iline){
	const double tt(sdInv * ray->Intersect(**iline));
	if((tt >= 0) && ((tMin < 0) || (tt < tMin))){
	  tMin = tt;
	}
      }
      delete ray;
    }
    // else don't touch tMin, invalidTime means "no collision"
    
    return tMin;
  }
  
  
  double DistanceObjective::
  CalculateValue(double measure, double floor)
  {
    return
      boundval(minValue,
	       minValue +
	       (measure - floor) * (maxValue - minValue) / (_maxTime - floor),
	       maxValue);
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
    for(ssize_t igy(_dimy - 1); igy >= 0; --igy){
      const double yy(FindYlength(igy));
      os << prefix;
      for(size_t igx(0); igx < _dimx; ++igx){
	const double xx(FindXlength(igx));
	if(_paddedHull->Contains(xx, yy))
	  os << ((*_grid)[igx][igy] ? "o" : ".");
	else{
	  if((*_grid)[igx][igy])
	    os << ((*_timeLookup)[igx][igy] ? "x" : "o");
	  else
	    os << ((*_timeLookup)[igx][igy] ? "-" : ".");
	}
      }
      os << "\n";
    }
  }
  
  
  double DistanceObjective::
  CollisionTime(size_t ix, size_t iy, size_t iqdl, size_t iqdr) const
  {
    if((*_timeLookup)[ix][iy])
      return (*_timeLookup)[ix][iy]->Get(iqdl, iqdr);
    return invalidTime;
  }
  
  
  size_t DistanceObjective::
  DimX() const
  {
    return _dimx;
  }
  
  
  size_t DistanceObjective::
  DimY() const
  {
    return _dimy;
  }
  
  
  bool DistanceObjective::
  CellOccupied(size_t ix, size_t iy) const
  {
    return (*_grid)[ix][iy];
  }

}
