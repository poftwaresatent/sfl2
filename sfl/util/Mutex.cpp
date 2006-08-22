/* 
 * Copyright (C) 2006 Roland Philippsen <roland dot philippsen at gmx dot net>
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


#include "Mutex.hpp"
#include "pdebug.hpp"
#include <pthread.h>


#define PDEBUG PDEBUG_ERR


using namespace boost;


namespace sfl {
  
  
  Mutex::
  Mutex(shared_ptr<pthread_mutex_t> mutex)
    : m_mutex(mutex)
  {
  }
  
  
  Mutex::
  ~Mutex()
  {
    if(EBUSY == pthread_mutex_destroy(m_mutex.get()))
      PDEBUG("WARNING still locked by another thread.\n");
  }
  
  
  shared_ptr<Mutex> Mutex::
  Create()
  {
    shared_ptr<Mutex> result;
    pthread_mutexattr_t attr;
    int status(pthread_mutexattr_init( & attr));
    if(0 != status)
      return result;
    status = pthread_mutexattr_settype( & attr, PTHREAD_MUTEX_RECURSIVE);
    if(0 != status)
      return result;
    shared_ptr<pthread_mutex_t> mutex(new pthread_mutex_t());
    status = pthread_mutex_init(mutex.get(), & attr);
    if(0 != status)
      return result;
    result.reset(new Mutex(mutex));
    return result;
  }
  
  
  bool Mutex::
  Lock()
  {
    return 0 == pthread_mutex_lock(m_mutex.get());
  }
  
  
  bool Mutex::
  TryLock()
  {
    return 0 == pthread_mutex_trylock(m_mutex.get());
  }
  
  
  bool Mutex::
  Unlock()
  {
    return 0 == pthread_mutex_unlock(m_mutex.get());
  }
    
}
