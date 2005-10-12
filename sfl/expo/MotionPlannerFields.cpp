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


#include "MotionPlannerFields.hpp"
#include "MotionPlannerState.hpp"


namespace expo {


  MotionPlannerFields::
  MotionPlannerFields(MotionController & _motionController,
		      sfl::DynamicWindow & _dynamicWindow,
		      const sfl::RobotModel & _robotModel,
		      sfl::BubbleBand & _bubbleBand,
		      const sfl::Odometry & _odometry):
    motionController(_motionController),
    dynamicWindow(_dynamicWindow),
    robotModel(_robotModel),
    bubbleBand(_bubbleBand),
    odometry(_odometry),
    null_state(new NullState(this)),
    take_aim_state(new TakeAimState(this)),
    aimed_state(new AimedState(this)),
    adjust_goal_heading_state(new AdjustGoalHeadingState(this)),
    at_goal_state(new AtGoalState(this)),
    goForward(true)
  {
  }


  MotionPlannerFields::
  ~MotionPlannerFields()
  {
    delete null_state;
    delete take_aim_state;
    delete aimed_state;
    delete adjust_goal_heading_state;
    delete at_goal_state;
  }

}
