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
#include <sfl/api/Goal.hpp>
#include <sfl/bband/BubbleList.hpp>
#include <boost/scoped_ptr.hpp>


namespace sfl {

  class GlobalScan;
  class Odometry;
  class RobotModel;
  class BubbleFactory;
  class ReplanHandler;
  
  
  /** \todo Some of the constant fields should be moved to BubbleList
      or the Parameters object. */
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
	       BubbleList::Parameters parameters);
    
    /** \todo Use smart pointers, AFTER figuring out all implications. */
    ~BubbleBand();
    
    
    void SetGoal(const Goal & global_goal);
    void SetNF1GoalRadius(double r);
    
    /** \note The GlobalScan object should be filtered, ie contain
	only valid readings. This can be obtained from
	Multiscanner::CollectScans() and
	Multiscanner::CollectGlobalScans(), whereas
	Scanner::GetScanCopy() can still contain readings that are out
	of range (represented as readings at the maximum rho
	value). */
    bool AppendGoal(const Goal & global_goal,
		    boost::shared_ptr<const GlobalScan> scan);
    bool AppendTarget(const Goal & global_goal);
    
    /** \note The GlobalScan object should be filtered, ie contain
	only valid readings. This can be obtained from
	Multiscanner::CollectScans() and
	Multiscanner::CollectGlobalScans(), whereas
	Scanner::GetScanCopy() can still contain readings that are out
	of range (represented as readings at the maximum rho
	value). */
    void Update(boost::shared_ptr<const GlobalScan> scan);
    
    /** \todo there's a hardcoded cutoff distance in here! */
    std::pair<double, double> GetSubGoal() const;
    
    /** \note Used for plotting. */
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
    const Odometry & m_odometry;
    
    boost::scoped_ptr<BubbleFactory> m_bubble_factory;
    boost::scoped_ptr<ReplanHandler> m_replan_handler;
    Frame m_frame;
    BubbleList * m_active_blist;
    
    const double m_reaction_radius;
    //    const double m_ignore_radius2;
    
    Goal m_global_goal;
    double m_nf1_goal_radius;	// not initialized? used?
    double m_min_ignore_distance; // not initialized? used?
    
    bool m_replan_request;
    state_t m_state;
    
    
    void UpdateRobotPose();
    void SetMinIgnoreDistance(double d);
  };

}

#endif // SUNFLOWER_BUBBLEBAND_HPP
