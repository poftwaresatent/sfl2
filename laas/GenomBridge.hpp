/* 
 * Copyright (C) 2006
 * Swiss Federal Institute of Technology, Zurich. All rights reserved.
 * 
 * Author: Roland Philippsen <roland dot philippsen at gmx dot net>
 *         Autonomous Systems Lab <http://www.asl.ethz.ch/>
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


#ifndef GENOM_BRIDGE_HPP
#define GENOM_BRIDGE_HPP

#include <posterLib.h>

namespace sfl {
  class Scanner;
}

namespace npm {
  class HAL;
}

class LAAS;

/**
   Forwards calls to the base class and simultaneously updates some
   LAAS-conformant Genom posters for interaction with the SFL
   module. Also some additional stuff.
*/
class GenomBridge
{
public:
  GenomBridge();
  ~GenomBridge();
  
  bool SetOdometry(double x, double y, double theta,
		   double sxx, double syy, double stt,
		   double sxy, double sxt, double syt);
  
  bool SetCurspeed(double sd, double thetad);

  bool SetGoal(double x, double y, double theta,
	       double dr, double dtheta, int via_goal);
  
  bool SetScan(const sfl::Scanner & scanner, npm::HAL & hal);

  bool GetSpeedref(double & sd, double & thetad, int & numRef);

private:
  POSTER_ID m_pompos, m_cartspeed, m_goal, m_curspeed, m_scanpolar;
};

#endif // GENOM_BRIDGE_HPP
