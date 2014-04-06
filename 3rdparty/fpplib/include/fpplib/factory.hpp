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

#ifndef FPPLIB_FACTORY_HPP
#define FPPLIB_FACTORY_HPP

#include <fpplib/registry.hpp>
#include <iosfwd>
#include <typeinfo>


namespace fpplib {
  
  using std::string;
  using std::ostream;
  
  
  /**
     Interface for the Creator objects that are used by Factory to
     create Configurable subclass instances based on their type
     name. Normally, it should not be necessary to derive your own
     custom classes from BaseCreator, or even the provided Creator<>
     template, because Factory::declare does all of the work for you..
   */
  class BaseCreator
  {
  public:
    virtual ~BaseCreator() {}
    virtual Configurable * create(string const & instance_name) = 0;
    virtual BaseRegistry & registry() = 0;
  };
  
  
  /**
     Sub-type-specific Creator. It is used by the Factory to manage
     its dictionary of Configurable sub types. Normally, your code
     should not need to be aware of this template, just call
     Factory::declare instead.
  */
  template<class SubType>
  class Creator
    : public BaseCreator
  {
  public:
    virtual SubType * create(string const & instance_name)
    {
      SubType * instance(new SubType(instance_name));
      registry_.add(instance_name, instance);
      return instance;
    }
    
    virtual Registry<SubType> & registry()
    {
      return registry_;
    }
    
  protected:
    Registry<SubType> registry_;
  };
  
  
  /**
     A Factory is an object that creates Configurable objects. You can
     register new types by calling Factory::declare, and then use
     Factory::create to allocate and construct new instances
     thereof. Due to the fact that every Creator maintains a registry
     of the instances that it created, you can also look up existing
     instances using Factory::find.
  */
  class Factory
  {
  public:
    virtual ~Factory();
    
    /**
       Register a new type of Configurable with the Factory, so that
       later it can be created using the type_name (thus, clients need
       not be aware of the subclass implementation, nor indeed of its
       class name).
       
       \note If the given type_name is already in use, the old
       declaration will be discarded and henceworth instances of the
       latest registered SubType will be returned by Factory::create.

       \todo Is it wise to silently replace previously existing type
       declarations?  Exceptions might come in handy here, but that
       may open up a can of worms...
    */
    template<class SubType>
    void declare(string const & type_name)
    {
      creator_t::iterator ic(creator_.find(type_name));
      if (ic == creator_.end()) {
	creator_.insert(make_pair(type_name, new Creator<SubType>()));
	type_code_to_name_.insert(make_pair(string(typeid(SubType).name()), type_name));
      }
      else {
	delete ic->second;
	ic->second = new Creator<SubType>();
      }
    }
    
    /**
       Create a new instance of a Configurable subclass, as previously
       registered using Factory::declare.
       
       \return A freshly constructed instance, or zero if the given
       type_name was not previously registered.
    */
    Configurable * create(string const & type_name,
			  string const & instance_name);

    /**
       Find an existing Configurable instance, given its type and
       instance names. See also the templatized find<> method.
       
       \return The existing instance, or zero if either the type or
       instance name did not match.

       \todo XXXX: kick this out, it is too cumbersome to need a type
       name for looking up instances. Also, take the SubType registry
       out of the Creators, just keep a single registry of
       configurables here in the factory, and hook instances into it
       before returning from create().
    */    
    Configurable * find(string const & type_name,
			string const & instance_name) const;
    
    /**
       Convenience method for finding instances by sub-type without
       needing to remember what type_name was used. Beware, however,
       that C++ name mangling is not aware of polymorphism. This means
       that if you declared something using a class, and then try to
       find it using one of its base classes, it will fail.
    */
    template<class SubType>
    SubType * find(string const & instance_name) const
    {
      dict_t::const_iterator id(type_code_to_name_.find(typeid(SubType).name()));
      if (type_code_to_name_.end() == id) {
	return 0;
      }
      return dynamic_cast<SubType*> (find (id->second, instance_name));
    }
    
    /**
       Alternative find method, which uses only the instance name. If
       there are instances of several types which have the same
       instance name, then the first one found is returned.
       
       \return The existing instance, or zero if the instance name did
       not match.
    */
    Configurable * find(string const & instance_name) const;
    
    BaseRegistry const * findRegistry(string const & type_name) const;
    
    template<class SubType>
    Registry<SubType> const * findRegistry() const
    {
      dict_t::const_iterator id(type_code_to_name_.find(typeid(SubType).name()));
      if (type_code_to_name_.end() == id) {
	return 0;
      }
      return dynamic_cast<Registry<SubType> const *>(findRegistry(id->second));
    }
    
    /**
       Debug method to write all the registered Configurable subclasses
       to the provided ostream.
    */
    void dump(string const & prefix, ostream & os) const;
    
    
  protected:
    typedef map<string, BaseCreator * > creator_t;
    creator_t creator_;
    typedef map<string, string> dict_t;
    dict_t type_code_to_name_; // XXXX to do: do we need a multimap here?
  };
  
}

#endif // FPPLIB_FACTORY_HPP
