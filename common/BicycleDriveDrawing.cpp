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

#include "BicycleDriveDrawing.hpp"
#include "wrap_gl.hpp"
#include <sfl/util/Frame.hpp>
#include <math.h>

#include <iostream>		// dbg

using namespace sfl;
using namespace boost;
using namespace std;


namespace npm {


  BicycleDriveDrawing::
  BicycleDriveDrawing(const string & name,
		      shared_ptr<const BicycleDrive> drive):
    Drawing(name),
    m_drive(drive),
    m_wheelbase(drive->wheelbase),
    m_wheelradius(drive->wheelradius),
    m_axlewidth(drive->axlewidth)
  {
  }


  void BicycleDriveDrawing::
  Draw()
  {
    shared_ptr<const sfl::Frame> trans(m_drive->PoseCache());
    double v_trans, steer;
    m_drive->GetState(v_trans, steer);
    
    cerr << "(" << trans->X() << "   " << trans->Y() << "   "
	 << trans->Theta() << ")   (" << v_trans << "   " << steer << ")\n";
    
    glBegin(GL_LINES);
    
    glMatrixMode(GL_MODELVIEW);
    glPushMatrix();
    glTranslated(trans->X(), trans->Y(), 0);
    glRotated(180 * trans->Theta() / M_PI, 0, 0, 1);
    
    glColor3d(1, 1, 1);
    glVertex2d(0, -m_axlewidth/2);
    glVertex2d(0,  m_axlewidth/2);

    glColor3d(1, 0.5, 0.5);
    glVertex2d(-m_wheelradius, -m_axlewidth/2);
    glVertex2d( m_wheelradius, -m_axlewidth/2);
    glVertex2d(-m_wheelradius,  m_axlewidth/2);
    glVertex2d( m_wheelradius,  m_axlewidth/2);
    
    glTranslated(m_wheelbase, 0, 0);
    glRotated(180 * steer / M_PI, 0, 0, 1);
    
    glColor3d(1, 1, 1);
    glVertex2d(0, -m_axlewidth/2);
    glVertex2d(0,  m_axlewidth/2);

    glColor3d(1, 0.5, 0.5);
    glVertex2d(-m_wheelradius, -m_axlewidth/2);
    glVertex2d( m_wheelradius, -m_axlewidth/2);
    glVertex2d(-m_wheelradius,  m_axlewidth/2);
    glVertex2d( m_wheelradius,  m_axlewidth/2);
  
    glPopMatrix();
    glEnd();
  }

}



