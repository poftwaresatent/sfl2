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
#include <sfl/api/GlobalScan.hpp>
#include <sfl/api/Odometry.hpp>
#include <sfl/api/RobotModel.hpp>
#include <sfl/bband/BubbleList.hpp>
#include <sfl/bband/BubbleFactory.hpp>
#include <sfl/bband/ReplanHandler.hpp>


namespace sfl {


  class BubbleBand
  {
  public:
    typedef enum { NOBAND, NEWBAND, VALIDBAND, UNSUREBAND } state_t;
    
    const BubbleList::Parameters parameters;
    
    
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
    int Update(boost::shared_ptr<const GlobalScan> scan);
    
    std::pair<double, double> GetSubGoal() const;

    /** \todo Used for plotting. */
    state_t State() const { return _state; }

    const Frame & RobotPose() const { return _frame; }
    const Goal & GlobalGoal() const { return _global_goal; }
    const BubbleList * ActiveBlist() const { return _active_blist; }

    /** \todo Used for plotting. */
    const ReplanHandler * GetReplanHandler() const {
      return & _replan_handler;
    }

    double RobotRadius() const { return _robot_radius; }
    double NF1GoalRadius() const { return _nf1_goal_radius; }
    double ReactionRadius() const { return _reaction_radius; }
    double MinIgnoreDistance() const { return _min_ignore_distance; }


  private:
    friend class BubbleList;
    //    friend class ReplanHandler;
    friend class MotionPlanner;

    const RobotModel & _robot_model;
    const Odometry & _odometry;

    BubbleFactory _bubble_factory;
    ReplanHandler _replan_handler;
    Frame _frame;
    BubbleList * _active_blist;

    double _robot_radius;
    double _robot_diameter;
    double _deletion_diameter;
    double _addition_diameter;
    double _reaction_radius;
    double _ignore_radius;
    double _ignore_radius2;

    Goal _global_goal;
    double _nf1_goal_radius;
    double _min_ignore_distance;

    bool _replan_request;
    state_t _state;

    void UpdateRobotPose();
    void SwapBubbleLists();
    void SetMinIgnoreDistance(double d);
  };

}

#endif // SUNFLOWER_BUBBLEBAND_HPP
