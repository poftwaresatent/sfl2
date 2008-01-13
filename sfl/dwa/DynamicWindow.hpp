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


//#include <sfl/util/numeric.hpp>
//#include <sfl/api/Scan.hpp>

#include <sfl/util/array2d.hpp>
#include <boost/shared_ptr.hpp>


namespace sfl {
  
  class RobotModel;
  class MotionController;
  class Scan;
  class DistanceObjective;  
  class HeadingObjective;  
  class SpeedObjective;  
  
  
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
  */
  class DynamicWindow
  {
  public:
    /**
       The speed and acceleration limits are defined in RobotModel.
       
       \note If you have legacy code that expects to pass a
       MotionController to this constructor, simply use a
       LegacyDynamicWindow instead.
    */
    DynamicWindow(int dimension,
		  double grid_width,
		  double grid_height,
		  double grid_resolution,
		  boost::shared_ptr<const RobotModel> robot_model,
		  //// If you expect this parameter, use a LegacyDynamicWindow
		  //// (see below)
		  //const MotionController & motion_controller,
		  double alpha_distance,
		  double alpha_heading,
		  double alpha_speed,
		  bool auto_init);
    
    bool Initialize(std::ostream * os, bool paranoid);
    
    bool Forbidden(int qdlIndex, int qdrIndex) const;
    bool Admissible(int qdlIndex, int qdrIndex) const;
    bool Reachable(int qdlIndex, int qdrIndex) const;
    
    /** \note The Scan object should be filtered, ie contain only
	valid readings. This can be obtained from
	Multiscanner::CollectScans(), whereas
	Scanner::GetScanCopy() can still contain readings that are out
	of range (represented as readings at the maximum rho
	value). */
    void Update(/** current left wheel speed (use LegacyDynamicWindow
		    if it's tricky for you to get that info) */
		double qdl,
		/** current right wheel speed */
		double qdr,
		/** (estimated or fixed) delay until next invocation */
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
    void _GoStrictFast();


    /** Makes the SpeedObjective attempt to keep the robot at standstill. */
    void GoSlow();
    void _GoStrictSlow();


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

    
    int Dimension() const { return dimension; }
    int QdlMinIndex() const { return m_qdlMin; }
    int QdlMaxIndex() const { return m_qdlMax; }
    int QdrMinIndex() const { return m_qdrMin; }
    int QdrMaxIndex() const { return m_qdrMax; }
    int QdlOptIndex() const { return m_qdlOpt; }
    int QdrOptIndex() const { return m_qdrOpt; }
    double ObjectiveMax() const { return m_objectiveMax; }
    double ObjectiveMin() const { return m_objectiveMin; }
    
    /** \pre index within 0..dimension */
    double Qd(int index) const { return m_qd[index]; }
    
    /** \pre indices within QdlMinIndex() ... QdrMaxIndex() */
    double Objective(int qdlIndex, int qdrIndex) const
    { return m_objective[qdlIndex][qdrIndex]; }
    
    boost::shared_ptr<DistanceObjective const> GetDistanceObjective() const
    { return m_distance_objective; }
    
    boost::shared_ptr<HeadingObjective const> GetHeadingObjective() const
    { return m_heading_objective; }

    boost::shared_ptr<SpeedObjective const> GetSpeedObjective() const
    { return m_speed_objective; }
    
    void DumpObstacles(std::ostream & os, const char * prefix) const;
    void DumpObjectives(std::ostream & os, const char * prefix) const;
    
    
    const int dimension;
    const int maxindex;
    const double resolution;
    const double qddMax;
    
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

    
    boost::shared_ptr<const RobotModel> m_robot_model;
    boost::shared_ptr<DistanceObjective> m_distance_objective;
    boost::shared_ptr<HeadingObjective> m_heading_objective;
    boost::shared_ptr<SpeedObjective> m_speed_objective;

    boost::scoped_array<double> m_qd; //[dimension];
    array2d<speedstate_t> m_state; //[dimension][dimension];
    array2d<double> m_objective; //[dimension][dimension];
    
    /** Used for colorscale when drawing internal info. */
    mutable double m_objectiveMax;
    /** Used for colorscale when drawing internal info. */
    mutable double m_objectiveMin;

    int m_qdlMin, m_qdlMax, m_qdrMin, m_qdrMax;
    mutable int m_qdlOpt;
    mutable int m_qdrOpt;
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
  
  
  /**
     Provides the "legacy" API where a MotionController was
     automatically used to get the current wheel speeds at each
     Update(). The problem with that approach was that it forces
     clients to (i) instantiate and manage a MotionController and (ii)
     call its Update() in synch.
  */
  class LegacyDynamicWindow
    : public DynamicWindow
  {
  public:
    LegacyDynamicWindow(int dimension,
			double grid_width,
			double grid_height,
			double grid_resolution,
			boost::shared_ptr<const RobotModel> robot_model,
			const MotionController & motion_controller,
			double alpha_distance,
			double alpha_heading,
			double alpha_speed,
			bool auto_init);
    
    /** Gets the current speeds from the registered MotionController
	and then forwards the call to DynamicWindow::Update(). This
	requires you to call MotionController::Update() in synch prior
	to this method (which is the way it is done in legacy
	code). */
    void Update(double timestep,
		double dx,
		double dy,
		boost::shared_ptr<const Scan> local_scan,
		std::ostream * dbgos = 0);
    
    const MotionController & motion_controller;
  };
  
}

#endif // SUNFLOWER_DYNAMICWINDOW_HPP
