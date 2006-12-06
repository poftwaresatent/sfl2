/* 
 * Copyright (C) 2006
 * Swiss Federal Institute of Technology, Zurich. All rights reserved.
 * 
 * Developed at the Autonomous Systems Lab.
 * Visit our homepage at http://www.asl.ethz.ch/
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


#ifndef NPM_PLAN_THREAD_HPP
#define NPM_PLAN_THREAD_HPP


#include <sfl/util/Pthread.hpp>
#include <sfl/gplan/GridFrame.hpp>


namespace estar {
  class Facade;
}


class PlanThread
  : public sfl::SimpleThread
{
public:
  PlanThread(boost::shared_ptr<estar::Facade> estar,
	     boost::shared_ptr<const sfl::GridFrame> gframe,
	     size_t grid_xsize, size_t grid_ysize);
  
  virtual void Step();
  
  /**
     \return 0 if there is a valid plan, 1 if the wavefront hasn't
     crossed the robot yet, 2 if there is no plan and the wavefront is
     empty or has already passed the robot (there is no path, the
     latter check is basically redundant because if there is no path
     the wavefront will never cross the robot), -1 if the robot is
     outside the grid, -2 if the robot is in an obstacle.
  */
  int GetStatus(double robot_global_x, double robot_global_y);
  
private:
  boost::shared_ptr<estar::Facade> m_estar;
  boost::shared_ptr<const sfl::GridFrame> m_gframe;
  const size_t m_grid_xsize;
  const size_t m_grid_ysize;
  boost::shared_ptr<sfl::Mutex> m_mutex;
};

#endif // NPM_PLAN_THREAD_HPP
