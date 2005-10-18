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


#include "Odometry.hpp"
#include "HAL.hpp"
#include <iostream>


using boost::shared_ptr;
using namespace std;


namespace sfl {
  
  
  Odometry::
  Odometry(HAL * hal):
    m_hal(hal)
  {
  }
  
  
  int Odometry::
  Init(const Pose & pose,
       ostream * dbgos)
  {
    // don't set timestamp here, will be read from HAL afterwards
    int res(m_hal->odometry_set(pose.X(), pose.Y(), pose.Theta(),
					  pose.Sxx(), pose.Syy(), pose.Stt(),
					  pose.Sxy(), pose.Sxt(), pose.Syt()));
    if(res != 0){
      if(dbgos != 0){
	(*dbgos) << "ERROR in Odometry::Init():\n"
		 << "  odometry_set() returned " << res << "\n";
      }
      return res;
    }
    
    struct ::timespec timestamp;
    res = m_hal->time_get(&timestamp);
    if(res != 0){
      if(dbgos != 0){
	(*dbgos) << "ERROR in Odometry::Init():\n"
		 << "  time_get() returned " << res << "\n";
      }
      return res;
    }
    
    m_history.clear();
    m_history.insert(make_pair(timestamp, shared_ptr<Pose>(new Pose(pose))));
    
    return 0;
  }
  
  
  int Odometry::
  Update(ostream * dbgos)
  {
    struct ::timespec timestamp;
    double x, y, t, sxx, syy, stt, sxy, sxt, syt;
    int res(m_hal->odometry_get(&timestamp,
				&x, &y, &t,
				&sxx, &syy, &stt,
				&sxy, &sxt, &syt));
    if(res != 0){
      if(dbgos != 0){
	(*dbgos) << "ERROR in Odometry::Update():\n"
		 << "  odometry_get() returned " << res << "\n";
      }
      return res;
    }
    
    m_history.insert(make_pair(timestamp,
			       shared_ptr<Pose>(new Pose(x, y, t,
							 sxx, syy, stt,
							 sxy, sxt, syt))));
    
    return 0;
  }
  
  
  const Pose & Odometry::
  Get()
    const
  {
    static const Pose null_pose;
    if(m_history.empty())
      return null_pose;
    return *(m_history.rbegin()->second);
  }
  
  
  const Pose * Odometry::
  Get(const Timestamp & t)
    const
  {
    history_t::const_iterator ih(m_history.find(t));
    if(ih == m_history.end())
      return 0;
    //       ostringstream os;
    //       os << "ERROR in sfl::Odometry::Get(const Timestamp &):\n"
    // 	 << "  no stamp for " << t << "\n"
    // 	 << "  available history:\n";
    //       for(i = _history.begin(); i != _history.end(); ++i)
    // 	os << "    " << i->first << ": " << i->second << "\n";
    return (ih->second).get();
  }
  
  
  int Odometry::
  Set(const Pose & pose)
  {
    // don't set timestamp here, use HAL's time function afterwards
    int res(m_hal->odometry_set(pose.X(), pose.Y(), pose.Theta(),
				pose.Sxx(), pose.Syy(), pose.Stt(),
				pose.Sxy(), pose.Sxt(), pose.Syt()));
    if(res != 0)
      return res;
      // error("sfl::Odometry::Set(): odometry_set()", res);
    
    struct ::timespec timestamp;
    res = m_hal->time_get(&timestamp);
    if(res != 0)
      return res;
      // error("sfl::Odometry::Set(): time_get()", res);
    m_history.insert(make_pair(timestamp, shared_ptr<Pose>(new Pose(pose))));
    
    return 0;
  }
  
}
