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


#include <boost/shared_ptr.hpp>
#include <string>


namespace sfl {
  class Scan;
}


namespace expo {
  
  
  class MotionPlanner;
  
  
  class MotionPlannerState
  {
  private:
    MotionPlannerState(const MotionPlannerState &); // non-copyable
    
  protected:
    MotionPlannerState(const std::string & name, MotionPlanner * mp);
  
  public:
    virtual ~MotionPlannerState();
  
    /** \note The Scan object should be filtered, ie contain only
	valid readings. This can be obtained from
	Multiscanner::CollectScans(), whereas Scanner::GetScanCopy()
	can still contain readings that are out of range (represented
	as readings at the maximum rho value). */
    virtual
    void Act(double timestep, boost::shared_ptr<const sfl::Scan> scan) = 0;
    virtual MotionPlannerState * NextState(double timestep);
    
    const std::string & Name() const;
    bool GoalReached() const;
    void GoForward(bool b);
    
    MotionPlannerState * GoalChangedState();
    MotionPlannerState * FollowTargetState();

  protected:
    typedef std::pair<double, double> direction_t;
    
    MotionPlanner * m_mp;
    const std::string m_name;

    virtual direction_t GetPathDirection() = 0;
    
    void TurnToward(double timestep, direction_t local_direction,
		    boost::shared_ptr<const sfl::Scan> scan) const;
    
    void GoAlong(double timestep, direction_t local_direction,
		 boost::shared_ptr<const sfl::Scan> scan) const;
    
    direction_t AskBubbleBand() const;
    
    void AskDynamicWindow(double timestep, direction_t local_direction,
			  boost::shared_ptr<const sfl::Scan> scan) const;
  };
  
  
  class TakeAimState
    : public MotionPlannerState
  {
  private:
    TakeAimState(const TakeAimState &); // non-copyable
    
  public:
    TakeAimState(MotionPlanner * mp);

    void Act(double timestep, boost::shared_ptr<const sfl::Scan> scan);
    MotionPlannerState * NextState(double timestep);

  protected:
    direction_t GetPathDirection();

  private:
    static const double DTHETASTARTHOMING;// =  10 * M_PI / 180;
    
    double dheading;
  
    bool StartHoming(double dtheta) const;
  };


  class AimedState
    : public MotionPlannerState
  {
  private:
    AimedState(const AimedState &); // non-copyable
    
  public:
    AimedState(MotionPlanner * mp);

    void Act(double timestep, boost::shared_ptr<const sfl::Scan> scan);
    MotionPlannerState * NextState(double timestep);

  protected:
    direction_t GetPathDirection();

  private:
    static const double DTHETASTARTAIMING;// = 45 * M_PI / 180;

    double dheading;

    bool StartAiming(double dtheta) const;
    bool Replan() const;
  };


  class AdjustGoalHeadingState
    : public MotionPlannerState
  {
  private:
    AdjustGoalHeadingState(const AdjustGoalHeadingState &); // non-copyable
    
  public:
    AdjustGoalHeadingState(MotionPlanner * mp);

    void Act(double timestep, boost::shared_ptr<const sfl::Scan> scan);

  protected:
    direction_t GetPathDirection();
  };


  class AtGoalState
    : public MotionPlannerState
  {
  private:
    AtGoalState(const AtGoalState &); // non-copyable
    
  public:
    AtGoalState(MotionPlanner * mp);

    void Act(double timestep, boost::shared_ptr<const sfl::Scan> scan);
    MotionPlannerState * NextState(double timestep);

  protected:
    direction_t GetPathDirection();
  };


  class NullState
    : public MotionPlannerState
  {
  private:
    NullState(const NullState &); // non-copyable
    
  public:
    NullState(MotionPlanner * mp);

    void Act(double timestep, boost::shared_ptr<const sfl::Scan> scan);
    MotionPlannerState * NextState(double timestep);

  protected:
    direction_t GetPathDirection();
  };

}

#endif // EXPO_MOTIONPLANNERSTATE_HPP
