/*
 * Copyright (c) 2005 CNRS/LAAS
 *
 * Author: Roland Philippsen <roland dot philippsen at gmx dot net>
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


#ifndef CWRAP_HANDLEMAP_HPP
#define CWRAP_HANDLEMAP_HPP


#include <boost/shared_ptr.hpp>
#include <set>
#include <map>


namespace sfl_cwrap {
  
  
  /**
     \note Totally exaggerated to use random IDs, but it was fun to
     implement. If this ever becomes a portability problem, just use
     monotonically incrementing IDs.
  */
  class IdPool
  {
  private:
    IdPool();
    IdPool(const IdPool & orig);
    
  public:
    ~IdPool();
    static boost::shared_ptr<IdPool> Instance();
    
    int Take();
    void Give(int id);
    
  private:
    std::set<int> m_pool;
    int m_urandom_fd;
  };
  
  
  template<class Pointee>
  class Handlemap
  {
  protected:
    struct value {
      value(boost::shared_ptr<Pointee> _pointee,
	    boost::shared_ptr<IdPool> _pool)
	: pointee(_pointee), pool(_pool) {}
      boost::shared_ptr<Pointee> pointee;
      boost::shared_ptr<IdPool> pool; // otherwise destroyed before me!
    };
    typedef std::map<int, value> map_t;
    map_t m_map;
    
  public:
    int Insert(boost::shared_ptr<Pointee> pointer){
      const int uid(IdPool::Instance()->Take());
      m_map.insert(std::make_pair(uid, value(pointer, IdPool::Instance())));
      return uid;
    }
    
    int InsertRaw(Pointee * raw_pointer)
    { return Insert(boost::shared_ptr<Pointee>(raw_pointer)); }
    
    boost::shared_ptr<Pointee> Find(int handle) const{
      typename map_t::const_iterator ih(m_map.find(handle));
      if(m_map.end() == ih)
	return boost::shared_ptr<Pointee>();
      return ih->second.pointee;
    }
    
    void Erase(int handle){
      typename map_t::iterator ih(m_map.find(handle));
      if(m_map.end() != ih){
	ih->second.pool->Give(handle);
	m_map.erase(ih);
      }
    }
  };

}

#endif // CWRAP_HANDLEMAP_HPP
