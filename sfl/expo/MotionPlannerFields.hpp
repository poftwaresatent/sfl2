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


#ifndef EXPO_MOTIONPLANNERFIELDS_HPP
#define EXPO_MOTIONPLANNERFIELDS_HPP


#include <sfl/api/Goal.hpp>
#include <sfl/api/Odometry.hpp>
#include <sfl/api/RobotModel.hpp>
#include <sfl/oa/expo/MotionController.hpp>
#include <sfl/oa/dwa/DynamicWindow.hpp>
#include <sfl/oa/bband/BubbleBand.hpp>


namespace expo {


  class MotionPlannerState;	// needed for circular dependency


  class MotionPlannerFields
  {
  public:
    MotionPlannerFields(MotionController & motionController,
			sfl::DynamicWindow & dynamicWindow,
			const sfl::RobotModel & robotModel,
			sfl::BubbleBand & bubbleBand,
			const sfl::Odometry & odometry);
    ~MotionPlannerFields();

    sfl::Goal goal;

    MotionController & motionController;
    sfl::DynamicWindow & dynamicWindow;
    const sfl::RobotModel & robotModel;
    sfl::BubbleBand & bubbleBand;
    const sfl::Odometry & odometry;
  
    MotionPlannerState * null_state;
    MotionPlannerState * take_aim_state;
    MotionPlannerState * aimed_state;
    MotionPlannerState * adjust_goal_heading_state;
    MotionPlannerState * at_goal_state;
  
    bool goForward;
    bool replan_request;		// kind of a hack
  };

}

#endif // EXPO_MOTIONPLANNERFIELDS_HPP
