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


#ifndef SUNFLOWER_MUTEX_HPP
#define SUNFLOWER_MUTEX_HPP


#include <boost/shared_ptr.hpp>


namespace sfl {
  
  
  /**
     Wraps around pthread_mutex_t, using PTHREAD_MUTEX_RECURSIVE
     semantics (see man pthread_mutexattr_init).
  */
  class Mutex
  {
  private:
    explicit Mutex(boost::shared_ptr<pthread_mutex_t> mutex);
    
  public:
    ~Mutex();
    
    /** Can return null if the system or process lacks resources. */
    boost::shared_ptr<Mutex> Create();
    
    /** Fails if a deadlock would occur. */
    bool Lock();
    
    /** Fails if already locked. */
    bool TryLock();
    
    /** Fails if not locked. */
    bool Unlock();
    
  private:
    boost::shared_ptr<pthread_mutex_t> m_mutex;
  };
  
}

#endif // SUNFLOWER_MUTEX_HPP
