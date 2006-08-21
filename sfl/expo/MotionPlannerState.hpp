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


#ifndef EXPO_MOTIONPLANNERSTATE_HPP
#define EXPO_MOTIONPLANNERSTATE_HPP


#include <sfl/expo/MotionPlannerFields.hpp>
#include <sfl/api/GlobalScan.hpp>
#include <cmath>
#include <string>
#include <utility>


namespace expo {


  class MotionPlannerState
  {
  private:
    MotionPlannerState(const MotionPlannerState &); // non-copyable
    
  protected:
    MotionPlannerState(const std::string & name,
		       MotionPlannerFields * fields);
  
  public:
    virtual ~MotionPlannerState();
  
    /** \note The GlobalScan object should be filtered, ie contain
	only valid readings. This can be obtained from
	Multiscanner::CollectScans() and
	Multiscanner::CollectGlobalScans(), whereas
	Scanner::GetScanCopy() can still contain readings that are out
	of range (represented as readings at the maximum rho
	value). */
    virtual void Act(double timestep,
		     boost::shared_ptr<const sfl::GlobalScan> global_scan) = 0;
    virtual MotionPlannerState * NextState(double timestep);

    const std::string & Name() const;
    bool GoalReached() const;
    void GoForward(bool b);

    MotionPlannerState * GoalChangedState();
    MotionPlannerState * FollowTargetState();

  protected:
    typedef std::pair<double, double> direction_t;

    MotionPlannerFields * _fields;
    std::string _name;

    /** \note The GlobalScan object should be filtered, ie contain
	only valid readings. This can be obtained from
	Multiscanner::CollectScans() and
	Multiscanner::CollectGlobalScans(), whereas
	Scanner::GetScanCopy() can still contain readings that are out
	of range (represented as readings at the maximum rho
	value). */
    virtual direction_t
    GetPathDirection(boost::shared_ptr<const sfl::GlobalScan> global_scan) = 0;
    
    /** \note The GlobalScan object should be filtered, ie contain
	only valid readings. This can be obtained from
	Multiscanner::CollectScans() and
	Multiscanner::CollectGlobalScans(), whereas
	Scanner::GetScanCopy() can still contain readings that are out
	of range (represented as readings at the maximum rho
	value). */
    void
    TurnToward(double timestep, direction_t local_direction,
	       boost::shared_ptr<const sfl::GlobalScan> global_scan) const;
    
    /** \note The GlobalScan object should be filtered, ie contain
	only valid readings. This can be obtained from
	Multiscanner::CollectScans() and
	Multiscanner::CollectGlobalScans(), whereas
	Scanner::GetScanCopy() can still contain readings that are out
	of range (represented as readings at the maximum rho
	value). */
    void
    GoAlong(double timestep, direction_t local_direction,
	    boost::shared_ptr<const sfl::GlobalScan> global_scan) const;
    
    /** \note The GlobalScan object should be filtered, ie contain
	only valid readings. This can be obtained from
	Multiscanner::CollectScans() and
	Multiscanner::CollectGlobalScans(), whereas
	Scanner::GetScanCopy() can still contain readings that are out
	of range (represented as readings at the maximum rho
	value). */
    direction_t
    AskBubbleBand(boost::shared_ptr<const sfl::GlobalScan> global_scan) const;
    
    /** \note The GlobalScan object should be filtered, ie contain
	only valid readings. This can be obtained from
	Multiscanner::CollectScans() and
	Multiscanner::CollectGlobalScans(), whereas
	Scanner::GetScanCopy() can still contain readings that are out
	of range (represented as readings at the maximum rho
	value). */
    void
    AskDynamicWindow(double timestep, direction_t local_direction,
		     boost::shared_ptr<const sfl::GlobalScan> global_scan)
      const;
  };
  
  
  class TakeAimState:
    public MotionPlannerState
  {
  private:
    TakeAimState(const TakeAimState &); // non-copyable
    
  public:
    TakeAimState(MotionPlannerFields * fields);

    void Act(double timestep,
	     boost::shared_ptr<const sfl::GlobalScan> global_scan);
    MotionPlannerState * NextState(double timestep);

  protected:
    direction_t
    GetPathDirection(boost::shared_ptr<const sfl::GlobalScan> global_scan);

  private:
    static const double DTHETASTARTHOMING =  10 * M_PI / 180;

    double dheading;
  
    bool StartHoming(double dtheta) const;
  };


  class AimedState:
    public MotionPlannerState
  {
  private:
    AimedState(const AimedState &); // non-copyable
    
  public:
    AimedState(MotionPlannerFields * fields);

    void Act(double timestep,
	     boost::shared_ptr<const sfl::GlobalScan> global_scan);
    MotionPlannerState * NextState(double timestep);

  protected:
    direction_t
    GetPathDirection(boost::shared_ptr<const sfl::GlobalScan> global_scan);

  private:
    static const double DTHETASTARTAIMING = 45 * M_PI / 180;

    double dheading;

    bool StartAiming(double dtheta) const;
    bool Replan() const;
  };


  class AdjustGoalHeadingState:
    public MotionPlannerState
  {
  private:
    AdjustGoalHeadingState(const AdjustGoalHeadingState &); // non-copyable
    
  public:
    AdjustGoalHeadingState(MotionPlannerFields * fields);

    void Act(double timestep,
	     boost::shared_ptr<const sfl::GlobalScan> global_scan);

  protected:
    direction_t
    GetPathDirection(boost::shared_ptr<const sfl::GlobalScan> global_scan);
  };


  class AtGoalState:
    public MotionPlannerState
  {
  private:
    AtGoalState(const AtGoalState &); // non-copyable
    
  public:
    AtGoalState(MotionPlannerFields * fields);

    void Act(double timestep,
	     boost::shared_ptr<const sfl::GlobalScan> global_scan);
    MotionPlannerState * NextState(double timestep);

  protected:
    direction_t
    GetPathDirection(boost::shared_ptr<const sfl::GlobalScan> global_scan);
  };


  class NullState:
    public MotionPlannerState
  {
  private:
    NullState(const NullState &); // non-copyable
    
  public:
    NullState(MotionPlannerFields * fields);

    void Act(double timestep,
	     boost::shared_ptr<const sfl::GlobalScan> global_scan);
    MotionPlannerState * NextState(double timestep);

  protected:
    direction_t
    GetPathDirection(boost::shared_ptr<const sfl::GlobalScan> global_scan);
  };

}

#endif // EXPO_MOTIONPLANNERSTATE_HPP
