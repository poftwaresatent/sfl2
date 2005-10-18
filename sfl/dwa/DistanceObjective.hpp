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


#ifndef SUNFLOWER_DISTANCEOBJECTIVE_HPP
#define SUNFLOWER_DISTANCEOBJECTIVE_HPP


#include <sfl/api/RobotModel.hpp>
#include <sfl/api/Scan.hpp>
#include <sfl/util/Hull.hpp>
#include <sfl/oa/dwa/Objective.hpp>
#include <sfl/oa/dwa/Lookup.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/scoped_array.hpp>
#include <boost/scoped_ptr.hpp>
#include <vector>


namespace sfl {


  /**
     The distance objective is what makes the dynamic window chose
     <ul>
     <li> which motion commands are forbidden </li>
     <li> which motion commands maximise clearance </li>
     </ul>
  
     It basically does 2 things: Predict collisions (and how far away
     they are), and translate these predictions into an objective value
     (typically between 0 and 1).

     All collision predictions are precalculated and stored in a lookup
     table. Apart from the traditional time-space tradeoff, this also
     implies an effectively "blown up" robot outline (the amount of
     enlargement depends on the grid resolution, it's on the order
     of sqrt(2) * resolution / 2).

     \note Don't get confused by the fact that there are two grids used
     here: One comes from Objectve (or rather, from DynamicWindow) and
     represents the actuator speed space of the robot. The other is a
     local obstacle grid attached to the robot and is a sampling of the
     workspace. The latter is (mostly) referred to with grid_WHATEVER.

     \todo The handling of border cells might still have the same
     problems as in the XOberon port. Check Initialize()!
  */
  class DistanceObjective:
    public Objective
  {
  public:
    DistanceObjective(const DynamicWindow & dynamic_window,
		      const RobotModel & robot_model,
		      double grid_width,
		      double grid_height,
		      double grid_resolution);
    
    
    void Initialize(std::ostream * progress_stream);
    void Calculate(unsigned int qdlMin,
		   unsigned int qdlMax,
		   unsigned int qdrMin,
		   unsigned int qdrMax,
		   boost::shared_ptr<const Scan> local_scan);
    void GetRange(double & x0, double & y0, double & x1, double & y1) const;

    int DimX() const { return _dimx; }
    int DimY() const { return _dimy; }
    bool CellOccupied(int ix, int iy) const { return _grid[ix][iy]; }

    /** \return -1 if not valid */
    double CollisionTime(int ix, int iy, int iqdl, int iqdr) const;
    
    /** \note Expects signed int to be consistent with FindXindex(). */
    double FindXlength(int i) const;

    /** \note Expects signed int to be consistent with FindYindex(). */
    double FindYlength(int i) const;

    /** \note Returns signed int to facilitate domain detection. */
    int FindXindex(double d) const;

    /** \note Returns signed int to facilitate domain detection. */
    int FindYindex(double d) const;

    boost::shared_ptr<const Hull> GetHull() const
    { return _hull; }
    boost::shared_ptr<const Hull> GetPaddedHull() const
    { return _paddedHull; }
    boost::shared_ptr<const Polygon> GetEvaluationHull() const
    { return _evaluationHull; }
    
    void DumpGrid(std::ostream & os, const char * prefix) const;


  protected:
    const double _securityDistance;
    const double _maxTime;
    const double _gridResolution;

    const RobotModel & _robot_model;

    std::vector<boost::shared_ptr<Line> > _outline;
    boost::shared_ptr<const Hull> _hull;
    boost::shared_ptr<const Hull> _paddedHull;
    boost::shared_ptr<const Polygon> _evaluationHull;
    
    boost::scoped_array<boost::scoped_array<bool> > _grid;
    double _x0, _y0, _x1, _y1; // bounding box of grid (currently symetric)
    double _dx, _dy, _dxInv, _dyInv; // effective resolution along x and y
    int _dimx, _dimy;		// dimensions of grid
    boost::scoped_array<double> _qdLookup;
    boost::scoped_array<boost::scoped_array<double> > _maxTimeLookup;
    boost::scoped_array<boost::scoped_array<boost::scoped_ptr<Lookup> > >
      _timeLookup;

    void ResetGrid();
    void UpdateGrid(boost::shared_ptr<const Scan> local_scan);
    double MinTime(unsigned int iqdl, unsigned int iqdr);

    double PredictCollision(double qdl, double qdr, double lx, double ly)
      const;
    
    double CalculateValue(double measure, double floor);
  };

}

#endif // SUNFLOWER_DISTANCEOBJECTIVE_HPP
