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


#ifndef SUNFLOWER_DYNAMICWINDOW_HPP
#define SUNFLOWER_DYNAMICWINDOW_HPP


#include <sfl/util/numeric.hpp>
#include <sfl/api/RobotModel.hpp>
#include <sfl/api/MotionController.hpp>
#include <sfl/api/Scan.hpp>
#include <sfl/dwa/DistanceObjective.hpp>
#include <sfl/dwa/HeadingObjective.hpp>
#include <sfl/dwa/SpeedObjective.hpp>


namespace sfl {


  /**
     \brief Main interface of ASL's DWA implementation.

     The dynamic window is based mostly on the original paper:
     D. Fox, W. Burgard, S. Thrun, "The Dynamic Window Approach to Collision
     Avoidance", IEEE Robotics & Automation Magazine, March 1997.

     For ASL specific modifications, consult the paper:
     R. Philippsen, R. Siegwart, "Smooth and Efficient Obstacle
     Avoidance for a Tour Guide Robot". In Proceedings of IEEE
     International Conference on Robotics and Automation, ICRA 2003,
     Taipei, Taiwan.

     In order to make it more modular, the sub-objectives are own
     classes, which use the some methods provided by the DynamicWindow.

     Basically, DynamicWindow provides access methods to the dynamic
     window and its flags and very basic dynamic model. The idea is to
     call Update() and then OptimalActuators() in order to know which
     motion command is best.

     Having moved the sub-objectives to their own classes makes it
     possible to more easily experiment with things such as
     look-up-tables for collision prediction.
     
     \todo time for code modernisation (smart ptrs and such)
  */
  class DynamicWindow
  {
  public:
    /** \note The speed and acceleration limits are defined in RobotModel. */
    DynamicWindow(int dimension,
		  double grid_width,
		  double grid_height,
		  double grid_resolution,
		  boost::shared_ptr<const RobotModel> robot_model,
		  const MotionController & motion_controller,
		  double alpha_distance,
		  double alpha_heading,
		  double alpha_speed,
		  bool auto_init);
    ~DynamicWindow();
    
    bool Initialize(std::ostream * os, bool paranoid);
    
    int Dimension() const;
    
    bool Forbidden(int qdlIndex, int qdrIndex) const;
    bool Admissible(int qdlIndex, int qdrIndex) const;
    bool Reachable(int qdlIndex, int qdrIndex) const;
    double Qd(int index) const;
    
    /** \note The Scan object should be filtered, ie contain only
	valid readings. This can be obtained from
	Multiscanner::CollectScans() and
	Multiscanner::CollectGlobalScans(), whereas
	Scanner::GetScanCopy() can still contain readings that are out
	of range (represented as readings at the maximum rho
	value). */
    void Update(/** (estimated or fixed) delay until next invocation */
		double timestep,
		/** local goal x component */
		double dx,
		/** local goal y component */
		double dy,
		boost::shared_ptr<const Scan> local_scan,
		std::ostream * dbgos = 0);
    
    void GetSubGoal(double & local_x, double & local_y) const;
    void SetHeadingOffset(double angle);
    double GetHeadingOffset() const;


    /** Makes the SpeedObjective maximise translational speeds. */
    void GoFast();
    void GoStrictFast();


    /** Makes the SpeedObjective attempt to keep the robot at standstill. */
    void GoSlow();
    void GoStrictSlow();


    /** Makes the SpeedObjective use forward speeds. Influences GoFast(). */
    void GoForward();


    /** Makes the SpeedObjective use backward speeds. Influences GoFast(). */
    void GoBackward();


    /**
       Choose best speed command, based on current alphaSpeed,
       alphaDistance, and alphaHeading. The values are returned through
       the double & parameters.
     
       \return true if optimal actuators were found, false
       otherwise. If this method returns false, the robot should
       probably perform an emergency stop ;-)
    */
    bool OptimalActuators(double & qdl, double & qdr) const;


    inline int QdlMinIndex() const;
    inline int QdlMaxIndex() const;
    inline int QdrMinIndex() const;
    inline int QdrMaxIndex() const;
    inline int QdlOptIndex() const;
    inline int QdrOptIndex() const;
    inline double ObjectiveMax() const;
    inline double ObjectiveMin() const;
    inline double Objective(int qdlIndex, int qdrIndex) const;

    inline const DistanceObjective & GetDistanceObjective() const;
    inline const HeadingObjective & GetHeadingObjective() const;
    inline const SpeedObjective & GetSpeedObjective() const;

    void DumpObstacles(std::ostream & os, const char * prefix) const;
    void DumpObjectives(std::ostream & os, const char * prefix) const;
    
    
    double alpha_distance;
    double alpha_heading;
    double alpha_speed;
    
    
  protected:
    /**
       Admissible speeds are all those that lie within
       <ul>
       <li> the motors' capabilities </li>
       <li> the chosen maximum translational and rotational speeds </li>
       </ul>

       Reachable speeds are those admissible speeds that can be reached given
       the current speed and the motors' acceleration (times the timestep).
     
       Forbidden are those speeds among the reachable set that would
       result in a collision (e.g. time to predicted collision is
       smaller than the time needed for a full stop).
    */
    typedef enum {
      ADMISSIBLE, REACHABLE, FORBIDDEN
    } speedstate_t;


    //RFCTR const double _reachableQd;

    const int _dimension;
    const int _maxindex;
    const double _resolution;
    
    boost::shared_ptr<const RobotModel> m_robot_model;
    const MotionController & _motion_controller;

    DistanceObjective _distance_objective;
    HeadingObjective _heading_objective;
    SpeedObjective _speed_objective;

    double * _qd;			//[dimension];
    speedstate_t ** _state;	//[dimension][dimension];
    double ** _objective;	//[dimension][dimension];

    /** Used for colorscale when drawing internal info. */
    mutable double _objectiveMax;
    /** Used for colorscale when drawing internal info. */
    mutable double _objectiveMin;

    int _qdlMin, _qdlMax, _qdrMin, _qdrMax;
    mutable int _qdlOpt;
    mutable int _qdrOpt;
    const double _qddMax;
    mutable bool m_compute_next_optimum;
    
    
    int FindIndex(double qd) const;
    double FindQd(int index) const;
    void InitForbidden();
    void CalculateReachable(double timestep, double qdl, double qdr);
    void CalculateAdmissible();
    void CalculateOptimum(double alphaDistance,
			  double alphaHeading,
			  double alphaSpeed) const;
  };


  int DynamicWindow::
  QdlMinIndex()
    const
  {
    return _qdlMin;
  }


  int DynamicWindow::
  QdlMaxIndex()
    const
  {
    return _qdlMax;
  }


  int DynamicWindow::
  QdrMinIndex()
    const
  {
    return _qdrMin;
  }


  int DynamicWindow::
  QdrMaxIndex()
    const
  {
    return _qdrMax;
  }


  int DynamicWindow::
  QdrOptIndex()
    const
  {
    return _qdrOpt;
  }


  int DynamicWindow::
  QdlOptIndex()
    const
  {
    return _qdlOpt;
  }


  double DynamicWindow::
  ObjectiveMax()
    const
  {
    return _objectiveMax;
  }


  double DynamicWindow::
  ObjectiveMin()
    const
  {
    return _objectiveMin;
  }


  double DynamicWindow::
  Objective(int qdlIndex,
	    int qdrIndex)
    const
  {
    return _objective[qdlIndex][qdrIndex];
  }


  const DistanceObjective & DynamicWindow::
  GetDistanceObjective()
    const
  {
    return _distance_objective;
  }


  const HeadingObjective & DynamicWindow::
  GetHeadingObjective()
    const
  {
    return _heading_objective;
  }


  const SpeedObjective & DynamicWindow::
  GetSpeedObjective()
    const
  {
    return _speed_objective;
  }


}

#endif // SUNFLOWER_DYNAMICWINDOW_HPP
