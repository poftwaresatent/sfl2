/* 
 * Copyright (C) 2004
 * Swiss Federal Institute of Technology, Lausanne. All rights reserved.
 * 
 * Author: Roland Philippsen <roland dot philippsen at gmx dot net>
 *         Autonomous Systems Lab <http://asl.epfl.ch/>
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


#include "ScannerDrawing.hpp"
#include <npm/RobotServer.hpp>
#include <npm/Lidar.hpp>
#include "wrap_glu.hpp"
#include <sfl/util/strutil.hpp>
#include <sfl/api/Scanner.hpp>
#include <sfl/api/Scan.hpp>


using namespace std;
using namespace sfl;


namespace npm {
  
  
  ScannerDrawing::
  ScannerDrawing(const Lidar * lidar)
    : Drawing(lidar->owner->GetName() + "_lidar_"
	      + to_string(lidar->GetScanner()->hal_channel),
	      "laser scanner data in global reference frame"),
      m_lidar(lidar)
  {
  }
  
  
  void ScannerDrawing::
  Draw()
  {
    vector<double> goodx, goody, failedx, failedy;
    const Scanner * scanner(m_lidar->GetScanner().get());
    const Frame & sensor_pose(m_lidar->GetGlobalPose());
    const Frame & robot_pose(m_lidar->owner->GetTruePose());
    glLineWidth(1);
    glColor3d(0.8, 0.4, 0);
    glBegin(GL_LINE_LOOP);
    glVertex2d(sensor_pose.X(), sensor_pose.Y());
    for(size_t is(0); is < m_lidar->nscans; ++is){
      scan_data data;
      const Scanner::status_t status(scanner->GetData(is, data));
      switch(status){
      case Scanner::SUCCESS:
	goodx.push_back(data.globx);
	goody.push_back(data.globy);
      case Scanner::OUT_OF_RANGE:
	glVertex2d(data.globx, data.globy);
	break;
      default:			// bug?
	{
	  double xx, yy;
	  scanner->CosPhi(is, xx);
	  scanner->SinPhi(is, yy);
	  m_lidar->mount->To(xx, yy);
	  robot_pose.To(xx, yy);
	  failedx.push_back(xx);
	  failedy.push_back(yy);
	  glVertex2d(xx, yy);
	}
      }
    }
    glEnd();
    
    if( ! goodx.empty()){
      glColor3d(1, 0.5, 0);
      glPointSize(3);
      glBegin(GL_POINTS);
      for(size_t ii(0); ii < goodx.size(); ++ii)
	glVertex2d(goodx[ii], goody[ii]);
      glEnd();
    }

    if( ! failedx.empty()){
      glColor3d(0, 0.5, 1);
      glPointSize(3);
      glBegin(GL_POINTS);
      for(size_t ii(0); ii < failedx.size(); ++ii)
	glVertex2d(failedx[ii], failedy[ii]);
      glEnd();
    }
  }
  
}
