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


#include "GridPlanner.hpp"
#include <cmath>



namespace sfl {



  GridPlanner::
  ~GridPlanner()
  {
  }



  void GridPlanner::
  Configure(position_t robot_position,
	    position_t global_goal,
	    double grid_width,
	    int grid_width_dimension)
  {
    _global_goal = global_goal;
    _global_home = robot_position;

    if(grid_width_dimension % 2 == 0)
      ++grid_width_dimension;

    double dx(global_goal.first  - robot_position.first);
    double dy(global_goal.second - robot_position.second);
    Frame frame(robot_position.first,
			    robot_position.second,
			    atan2(dy, dx));

    double width_offset
      = 0.5 * grid_width * (grid_width_dimension - 1)
      / grid_width_dimension;
    double delta = grid_width / grid_width_dimension;
    double x_frame = - width_offset;
    double y_frame = - width_offset;
    frame.To(x_frame, y_frame);

    _frame.Configure(x_frame, y_frame, frame.Theta(), delta);

    _goal_index = _frame.GlobalIndex(global_goal);
    _home_index = _frame.GlobalIndex(_global_home);

    _grid_dimension.first =
      (int) ceil((sqrt(dx*dx+dy*dy) + grid_width) / delta);

    _grid_dimension.second = grid_width_dimension;
  }



}
