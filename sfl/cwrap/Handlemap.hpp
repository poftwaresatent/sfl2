/*
 * Copyright (c) 2005 CNRS/LAAS
 *
 * Author: Roland Philippsen <roland.philippsen@gmx.net>
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
#include <map>


namespace sfl_cwrap {
  
  
  template<class Pointee>
  class Handlemap
  {
  protected:
    typedef std::map<int, boost::shared_ptr<Pointee> > map_t;
    map_t m_map;
    
  public:
    
    int Insert(boost::shared_ptr<Pointee> pointer)
    {
      if(m_map.empty())
	m_map.insert(std::make_pair(0, pointer));
      else
	m_map.insert(std::make_pair(m_map.rbegin()->first + 1, pointer));
      return m_map.rbegin()->first;
    }
    
    int InsertRaw(Pointee * raw_pointer)
    { return Insert(boost::shared_ptr<Pointee>(raw_pointer)); }
    
    
    boost::shared_ptr<Pointee> Find(int handle)
      const
    {
      typename map_t::const_iterator ih(m_map.find(handle));
      if(m_map.end() == ih)
	return boost::shared_ptr<Pointee>();
      return ih->second;
    }
    
    
    void Erase(int handle)
    {
      typename map_t::iterator ih(m_map.find(handle));
      if(m_map.end() != ih)
	m_map.erase(ih);
    }
    
  };

}

#endif // CWRAP_HANDLEMAP_HPP
