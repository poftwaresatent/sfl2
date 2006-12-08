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


#ifndef GENOM_HAL_HPP
#define GENOM_HAL_HPP


#include <npm/common/HAL.hpp>
#include <boost/shared_ptr.hpp>
#include <posterLib.h>


namespace sfl {
  class Mutex;
  class RobotModel;
}


class LAAS;


/**
   Forwards calls to the base class and simultaneously updates some
   LAAS-conformant Genom posters for interaction with the SFL
   module. Also some additional stuff.
*/
class GenomHAL
  : public npm::HAL
{
public:
  GenomHAL(npm::RobotServer * owner, LAAS * laas);
  virtual ~GenomHAL();
  
  virtual int odometry_set(double x, double y, double theta,
			   double sxx, double syy, double stt,
			   double sxy, double sxt, double syt);
  virtual int speed_get(double * qdl, double * qdr);
  
  int goal_set(double x, double y, double theta,
	       double dr, double dtheta, int via_goal);
  
  void UpdateScan();
  
protected:
  virtual void UpdateSpeeds();
  
private:
  LAAS * m_laas;
  boost::shared_ptr<sfl::Mutex> m_mutex;
  POSTER_ID m_pompos;
  POSTER_ID m_scanpolar;
  POSTER_ID m_cartspeed;
  POSTER_ID m_curspeed;
  POSTER_ID m_goal;
};


class GenomHALFactory
  : public npm::HALFactory
{
public:
  GenomHALFactory(LAAS * laas);
  virtual GenomHAL * Create(npm::RobotServer * owner) const;
  
private:
  LAAS * m_laas;
};

#endif // GENOM_HAL_HPP
