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


#include "TrajectoryDrawing.hpp"
#include "RobotServer.hpp"
#include "wrap_gl.hpp"
#include "pdebug.hpp"
#include "Manager.hpp"
#include <sfl/util/Frame.hpp>


namespace npm {
  
  
  TrajectoryDrawing::
  TrajectoryDrawing(const RobotServer * owner)
    : Drawing(owner->GetName() + "_trajectory",
	      Instance<UniqueManager<Drawing> >()),
    m_owner(owner)
  {
  }
  
  
  void TrajectoryDrawing::
  Draw()
  {
    const RobotServer::trajectory_t * traj(m_owner->GetNoisyTrajectory());
    if(traj && ( ! traj->empty())){
      PDEBUG_OUT("noisy trajectory baby!\n");
      glColor3d(0.6, 0.2, 0.2);
      if(traj->size() == 1){
	glPointSize(3);
	glBegin(GL_POINTS);
	glVertex2d((*traj)[0]->X(), (*traj)[0]->Y());
	glEnd();
	return;
      }
      glBegin(GL_LINE_STRIP);
      for(unsigned int i(0); i < traj->size(); ++i)
	glVertex2d((*traj)[i]->X(), (*traj)[i]->Y());
      glEnd();
    }
    
    traj = & m_owner->GetTrueTrajectory();
    if(traj->empty())
      return;
    glColor3d(1, 0.3, 0.3);
    if(traj->size() == 1){
      glPointSize(3);
      glBegin(GL_POINTS);
      glVertex2d((*traj)[0]->X(), (*traj)[0]->Y());
      glEnd();
      return;
    }
    glBegin(GL_LINE_STRIP);
    for(unsigned int i(0); i < traj->size(); ++i)
      glVertex2d((*traj)[i]->X(), (*traj)[i]->Y());
    glEnd();
  }
  
}
