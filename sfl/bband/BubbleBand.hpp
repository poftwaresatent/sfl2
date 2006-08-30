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


#ifndef SUNFLOWER_BUBBLEBAND_HPP
#define SUNFLOWER_BUBBLEBAND_HPP


#include <sfl/util/Frame.hpp>
#include <sfl/util/Pthread.hpp>
#include <sfl/api/Goal.hpp>
#include <sfl/bband/BubbleList.hpp>
#include <boost/scoped_ptr.hpp>
#include <boost/shared_ptr.hpp>


namespace sfl {

  class Scan;
  class Odometry;
  class RobotModel;
  class BubbleFactory;
  class ReplanHandler;
  class Multiscanner;
  
  /**
     Optional update thread for BubbleBand.
  */
  class BubbleBandThread
    : public SimpleThread
  {
  private:
    BubbleBandThread(const BubbleBandThread &);
    
  public:
    /** You still have to call BubbleBand::SetThread() and
	BubbleBandThread::Start(). */
    BubbleBandThread(const std::string & name);
    virtual void Step();
    
  protected:
    friend class BubbleBand;
    BubbleBand * bubbleBand;
  };
  
  
  /**
     \todo Some of the constant fields should be moved to BubbleList
     or the Parameters object.
  */
  class BubbleBand
  {
  public:
    typedef enum {
      NOBAND,
      NEWBAND,
      VALIDBAND,
      UNSUREBAND
    } state_t;
    
    const BubbleList::Parameters parameters;
    const double robot_radius;
    const double robot_diameter; // for BubbleList
    const double ignore_radius; // for BubbleList
    const double deletion_diameter; // for BubbleList
    const double addition_diameter; // for BubbleList
    
    
    BubbleBand(const RobotModel & robot_model,
	       const Odometry & odometry,
	       const Multiscanner & multiscanner,
	       BubbleList::Parameters parameters,
	       boost::shared_ptr<RWlock> rwlock);
    
    /** \todo Use smart pointers, AFTER figuring out all implications. */
    ~BubbleBand();
    
    void Update();
    
    /** Attempt to attach an update thread. Fails if this BubbleBand
	already has an update thread. */
    bool SetThread(boost::shared_ptr<BubbleBandThread> thread);
    
    void SetGoal(const Goal & global_goal);
    
    /** \note The Scan object should be filtered, ie contain only
	valid readings. This can be obtained from
	Multiscanner::CollectScans(), whereas Scanner::GetScanCopy()
	can still contain readings that are out of range (represented
	as readings at the maximum rho value). */
    bool AppendGoal(const Goal & global_goal,
		    boost::shared_ptr<const Scan> scan);
    bool AppendTarget(const Goal & global_goal);
    
    /** \note (goalx, goaly) might actually be the global goal, for
	instance if no band exists or if no bubble lies farther than
	carrot_distance from the root bubble */
    void GetSubGoal(double carrot_distance,
		    double & goalx, double & goaly) const;
    
    state_t GetState() const { return m_state; }
    const Frame & RobotPose() const { return m_frame; }
    const Goal & GlobalGoal() const { return m_global_goal; }
    const BubbleList * ActiveBlist() const { return m_active_blist; }
    
    /** \note Used for plotting. */
    const ReplanHandler * GetReplanHandler() const
    { return m_replan_handler.get(); }
    
    double NF1GoalRadius() const { return m_nf1_goal_radius; }
    double ReactionRadius() const { return m_reaction_radius; }
    double MinIgnoreDistance() const { return m_min_ignore_distance; }
    
    
  private:
    friend class BubbleBandThread;
    
    /** \note The GlobalScan object should be filtered, ie contain
	only valid readings. This can be obtained from
	Multiscanner::CollectScans() and
	Multiscanner::CollectGlobalScans(), whereas
	Scanner::GetScanCopy() can still contain readings that are out
	of range (represented as readings at the maximum rho
	value). */
    void DoUpdate(boost::shared_ptr<const Scan> scan);
    
    const Odometry & m_odometry;
    const Multiscanner & m_multiscanner;
    
    boost::scoped_ptr<BubbleFactory> m_bubble_factory;
    boost::scoped_ptr<ReplanHandler> m_replan_handler;
    Frame m_frame;
    BubbleList * m_active_blist;
    const double m_reaction_radius;
    Goal m_global_goal;
    double m_nf1_goal_radius;	// used by replan handler to init NF1

    /** \todo unused? (maybe when tracking moving targets...) */
    double m_min_ignore_distance; // used by replan handler to init bubbles
    bool m_replan_request;
    state_t m_state;
    boost::shared_ptr<RWlock> m_rwlock;
    boost::shared_ptr<BubbleBandThread> m_thread;
  };

}

#endif // SUNFLOWER_BUBBLEBAND_HPP
