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


////#include <sfl/util/Pthread.hpp>
#include <sfl/gplan/GridFrame.hpp>
#include <boost/shared_ptr.hpp>


namespace estar {
  class Facade;
}


class PlanThread
////  : public sfl::SimpleThread
{
public:
  typedef enum {
    /** there is a valid plan */
    HAVE_PLAN = 0,
    /** the wavefront hasn't crossed the robot yet */
    PLANNING = 1,
    /** there is no plan and the wavefront is empty */
    UNREACHABLE = 2,
    /** whatevers else, robot is at a goal node */
    AT_GOAL = 3,
    /** the robot is outside the grid */
    OUT_OF_GRID = -1,
    /** the robot is in an obstacle */
    IN_OBSTACLE = -2,
    /** an error has occurred (probably a bug) */
    ERROR = -42
  } status_t;
  
  
  PlanThread(boost::shared_ptr<estar::Facade> estar,
	     const sfl::GridFrame & gframe,
	     size_t grid_xsize, size_t grid_ysize);
  
  ////  virtual void Step();
  void Step();
  
  status_t GetStatus(double robot_global_x, double robot_global_y);
  
private:
  boost::shared_ptr<estar::Facade> m_estar;
  const sfl::GridFrame m_gframe;
  const size_t m_grid_xsize;
  const size_t m_grid_ysize;
  ////  boost::shared_ptr<sfl::Mutex> m_mutex;
};

#endif // NPM_PLAN_THREAD_HPP
