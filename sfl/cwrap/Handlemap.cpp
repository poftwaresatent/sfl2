/*
 * Copyright (c) 2006 Roland Philippsen <roland dot philippsen at gmx dot net>
 *
 * Permission to use, copy, modify, and distribute this software for any
 * purpose with or without fee is hereby granted, provided that the above
 * copyright notice and this permission notice appear in all copies.
 *
 * THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
 * WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
 * ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
 * WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
 * ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
 * OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
 */


#include "Handlemap.hpp"
#include <sfl/util/pdebug.hpp>
#include <iostream>
#include <fcntl.h>


#ifdef SFL_DEBUG
# define PDEBUG PDEBUG_ERR
# define PVDEBUG PDEBUG_OFF
#else // ! SFL_DEBUG
# define PDEBUG PDEBUG_OFF
# define PVDEBUG PDEBUG_OFF
#endif // SFL_DEBUG


using namespace boost;
using namespace std;


namespace sfl_cwrap {
  
  
  IdPool::
  IdPool()
    : m_urandom_fd(open("/dev/urandom", O_RDONLY, 0))
  {
    if(0 > m_urandom_fd){
      perror("/dev/urandom");
      abort();
    }
    PVDEBUG("ctor\n");
  }
  
  
  IdPool::
  ~IdPool()
  {
    close(m_urandom_fd);
    PVDEBUG("dtor\n");
  }
  
  
  shared_ptr<IdPool> IdPool::
  Instance()
  {
    static shared_ptr<IdPool> instance;
    if( ! instance){
      PVDEBUG("reset\n");
      instance.reset(new IdPool());
    }
    PVDEBUG("instance 0x%08X\n", instance.get());
    return instance;
  }
  
  
  int IdPool::
  Take()
  {
    int uid;
    while(true){
      if(sizeof(uid) != read(m_urandom_fd, &uid, sizeof(uid))){
	perror("read(/dev/urandom)");
	abort();
      }
      if(uid < 0)
	uid = -uid;
      if(m_pool.end() == m_pool.find(uid))
	break;
    }
    m_pool.insert(uid);
    PVDEBUG("%d\n", uid);
    return uid;
  }
  
  
  void IdPool::
  Give(int id)
  {
    PVDEBUG("%d\n", id);
    set<int>::iterator ii(m_pool.find(id));
    if(m_pool.end() != ii)
      m_pool.erase(ii);
    else
      PVDEBUG("bizarre, not in pool...\n");
  }

}
