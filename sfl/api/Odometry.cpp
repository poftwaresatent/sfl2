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
#include "Pose.hpp"
#include <iostream>


using namespace boost;
using namespace std;


namespace sfl {
  
  
  OdometryThread::
  OdometryThread(const string & name, ostream * _dbgos)
    : SimpleThread(name), dbgos(_dbgos)
  {
  }
  
  
  void OdometryThread::
  Step()
  {
    if( ! odometry){
      update_status = -42;
      return;
    }
    update_status = odometry->DoUpdate(dbgos);
  }
  
  
  Odometry::
  Odometry(shared_ptr<HAL> hal, shared_ptr<RWlock> rwlock)
    : m_hal(hal), m_rwlock(rwlock)
  {
  }
  
  
  int Odometry::
  Init(const Pose & pose, ostream * dbgos)
  {
    // don't set timestamp here, will be read from HAL afterwards
    int res(m_hal->odometry_set(pose.X(), pose.Y(), pose.Theta(),
				pose.Sxx(), pose.Syy(), pose.Stt(),
				pose.Sxy(), pose.Sxt(), pose.Syt()));
    if(res != 0){
      if(dbgos != 0)
	(*dbgos) << "ERROR in Odometry::Init():\n"
		 << "  odometry_set() returned " << res << "\n";
      return res;
    }
    struct ::timespec timestamp;
    res = m_hal->time_get(&timestamp);
    if(res != 0){
      if(dbgos != 0)
	(*dbgos) << "ERROR in Odometry::Init():\n"
		 << "  time_get() returned " << res << "\n";
      return res;
    }
    m_rwlock->Wrlock();
    m_history.clear();
    m_history.insert(make_pair(timestamp, shared_ptr<Pose>(new Pose(pose))));
    m_rwlock->Unlock();
    return 0;
  }
  
  
  int Odometry::
  Update(ostream * dbgos)
  {
    if(m_thread){
      m_thread->dbgos = dbgos;
      return m_thread->update_status;
    }
    return DoUpdate(dbgos);
  }
  
  
  int Odometry::
  DoUpdate(ostream * dbgos)
  {
    struct ::timespec timestamp;
    double x, y, t, sxx, syy, stt, sxy, sxt, syt;
    int res(m_hal->odometry_get(&timestamp,
				&x, &y, &t,
				&sxx, &syy, &stt,
				&sxy, &sxt, &syt));
    if(res != 0){
      if(dbgos != 0)
	(*dbgos) << "ERROR in Odometry::Update():\n"
		 << "  odometry_get() returned " << res << "\n";
      return res;
    }
    shared_ptr<Pose> pose(new Pose(x, y, t, sxx, syy, stt, sxy, sxt, syt));
    m_rwlock->Wrlock();
    m_history.insert(make_pair(timestamp, pose));    
    m_rwlock->Unlock();
    return 0;
  }
  
  
  shared_ptr<const Pose> Odometry::
  Get() const
  {
    RWlock::rdsentry sentry(m_rwlock);
    if(m_history.empty())
      return shared_ptr<const Pose>(new Pose());
    return m_history.rbegin()->second;
  }
  
  
//   const Pose * Odometry::
//   Get(const Timestamp & t)
//     const
//   {
//     history_t::const_iterator ih(m_history.find(t));
//     if(ih == m_history.end())
//       return 0;
//     //       ostringstream os;
//     //       os << "ERROR in sfl::Odometry::Get(const Timestamp &):\n"
//     // 	 << "  no stamp for " << t << "\n"
//     // 	 << "  available history:\n";
//     //       for(i = _history.begin(); i != _history.end(); ++i)
//     // 	os << "    " << i->first << ": " << i->second << "\n";
//     return (ih->second).get();
//   }
  
  
  int Odometry::
  Set(const Pose & pose)
  {
    // don't set timestamp here, use HAL's time function afterwards
    int res(m_hal->odometry_set(pose.X(), pose.Y(), pose.Theta(),
				pose.Sxx(), pose.Syy(), pose.Stt(),
				pose.Sxy(), pose.Sxt(), pose.Syt()));
    if(res != 0)
      return res;
    struct ::timespec timestamp;
    res = m_hal->time_get(&timestamp);
    if(res != 0)
      return res;
    m_rwlock->Wrlock();
    m_history.insert(make_pair(timestamp, shared_ptr<Pose>(new Pose(pose))));
    m_rwlock->Unlock();
    return 0;
  }
  
  
  bool Odometry::
  SetThread(shared_ptr<OdometryThread> thread)
  {
    RWlock::wrsentry sentry(m_rwlock);
    if(m_thread)
      return false;
    m_thread = thread;
    thread->odometry = this;
    return true;
  }
  
}
