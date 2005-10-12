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


#ifndef SUNFLOWER_SPEEDOBJECTIVE_HPP
#define SUNFLOWER_SPEEDOBJECTIVE_HPP



#include <sfl/api/RobotModel.hpp>
#include <sfl/oa/dwa/Objective.hpp>
#include <sfl/oa/dwa/Lookup.hpp>



namespace sfl {



class SpeedObjective:
  public Objective
{
public:
  SpeedObjective(const DynamicWindow & dynamic_window,
		 const RobotModel & robot_model);
  virtual ~SpeedObjective();

  void Initialize(std::ostream * progress_stream);
  void Calculate(unsigned int qdlMin,
		 unsigned int qdlMax,
		 unsigned int qdrMin,
		 unsigned int qdrMax);
  
  void GoFast();
  void GoSlow();
  void GoForward();
  void GoBackward();



protected:
  const double _sdMax;

  const RobotModel & _robot_model;

  double ** _sd;		//[dimension][dimension];
  double ** _thetad;		//[dimension][dimension];
  bool _goForward;

  Lookup _forward, _backward, _slow;
  Lookup * _current;
};



}

#endif // SUNFLOWER_SPEEDOBJECTIVE_HPP
