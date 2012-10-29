/*
 * fpplib - Factory and Parameter Parsing Library
 *
 * Copyright (c) 2011 Roland Philippsen. All rights reserved.
 *
 * BSD license:
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holder nor the names of
 *    contributors to this software may be used to endorse or promote
 *    products derived from this software without specific prior written
 *    permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHORS AND CONTRIBUTORS ``AS IS''
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED
 * TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
 * PARTICULAR PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT
 * HOLDER OR THE CONTRIBUTORS TO THIS SOFTWARE BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef FPPLIB_REGISTRY_HPP
#define FPPLIB_REGISTRY_HPP

#include <string>
#include <map>
#include <vector>


namespace fpplib {
  
  using std::string;
  using std::map;
  using std::vector;


  class Configurable;
  
  
  template<typename instance_type>
  class InstanceRegistry
  {
  public:
    typedef map<string, instance_type> map_t;
    
    virtual ~InstanceRegistry() {}
    
    void add(string const & instance_name,
	     instance_type instance)
    {
      map_[instance_name] = instance;
      vector_.push_back(instance);
    }
    
    instance_type find(string const & instance_name) const
    {
      typename map_t::const_iterator ii(map_.find(instance_name));
      if (map_.end() != ii) {
	return ii->second;
      }
      return nullInstance();
    }
    
    inline size_t size() const
    {
      return vector_.size();
    }
    
    inline instance_type at(size_t index) const
    {
      return vector_[index];
    }
    
  //   inline map_t map__() const
  //   {
  //     return map_;
  //   }
    
  // protected:
    typedef vector<instance_type> vector_t;
    
    map_t map_;
    vector_t vector_;
    
    virtual instance_type nullInstance() const
    {
      instance_type it;
      return it;
    }
  };
  
  
  template<typename pointer_type>
  class PointerRegistry
    : public InstanceRegistry<pointer_type>
  {
  public:
    virtual ~PointerRegistry()
    {
      for (size_t ii(0); ii < InstanceRegistry<pointer_type>::vector_.size(); ++ii) {
	delete InstanceRegistry<pointer_type>::vector_[ii];
      }
    }

  protected:
    virtual pointer_type nullInstance() const
    {
      return 0;
    }
  };
  
  
  class BaseRegistry
  {
  public:
    virtual ~BaseRegistry() {}
    
    virtual Configurable * find(string const & instance_name) const = 0;
    
    virtual size_t size() const = 0;
    
    virtual Configurable * at(size_t index) const = 0;
  };
  
  
  template<class SubType>
  class Registry
    : public BaseRegistry
  {
  public:
    void add(string const & instance_name,
	     SubType * instance)
    {
      imp_.add(instance_name, instance);
    }
    
    virtual SubType * find(string const & instance_name) const
    {
      return imp_.find(instance_name);
    }
    
    virtual size_t size() const
    {
      return imp_.size();
    }
    
    virtual SubType * at(size_t index) const
    {
      return imp_.at(index);
    }
    
  protected:
    PointerRegistry<SubType * > imp_;
  };
  
}

#endif // FPPLIB_REGISTRY_HPP
