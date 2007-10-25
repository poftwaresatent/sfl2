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


#ifndef NPM_MANAGER_HPP
#define NPM_MANAGER_HPP


#include <boost/shared_ptr.hpp>
#include <boost/enable_shared_from_this.hpp>
#include <string>
#include <map>
#include <iosfwd>


namespace npm {
  
  
  class Manageable;
  
  
  /**
     Managers are collections of named entities (subclasses of
     Manageable). Entities with empty names (i.e. "") are ignored to
     make it easier to handle transient objects.
  */
  class Manager
  {
  public:
    virtual ~Manager() {}
    
    /**
       \note entries with empty names "" are silently ignored
       
       \return any (different or same) entity previously attached
       under the same name
    */
    Manageable * Attach(Manageable * entry);
    
    /**
       \note entries with empty names "" silently return true
       \return false if the named entry mismatches the one in the
       catalog or if no entry exists under that name
    */
    bool Detach(const Manageable * entry);
    
    /** \return if it exists, the entity registered under the given name */
    Manageable * Retrieve(const std::string & name) const;
    
    void PrintCatalog(std::ostream & os) const;
    
    /** Apply a functor to all managed objects which are of the
	specified Subclass type (specializing by subclass allows
	Walker to use e.g. method overloading). The Walker class has
	to provide a method 'void Walk(Subclass *)'. */
    template<class Walker, class Subclass>
    void Walk(Walker walker) {
      for(catalog_t::iterator ic(catalog.begin()); ic != catalog.end(); ++ic) {
	Subclass * sc(dynamic_cast<Subclass *>(ic->second));
	if (sc)
	  walker.Walk(sc);
      }
    }
    
  protected:
    typedef std::map<std::string, Manageable *> catalog_t;
    catalog_t catalog;
  };
  
  
  template<class Subclass>
  class SubManager
    : public Manager
  {
  public:
    Subclass * Attach(Subclass * entry)
    { return dynamic_cast<Subclass *>(Manager::Attach(entry)); }
    
    Subclass * Retrieve(const std::string & name) const
    { return dynamic_cast<Subclass *>(Manager::Retrieve(name)); }
    
    template<class Walker>
    void Walk(Walker walker)
    { Manager::Walk<Walker, Subclass>(walker); }
  };
  
  
  template<class MgrSingleton>
  boost::shared_ptr<MgrSingleton> Instance();
  
  
  template<class Subclass>
  class UniqueManager
    : public SubManager<Subclass>
  {
  private:
    friend boost::shared_ptr<UniqueManager> Instance<UniqueManager>();
    UniqueManager() {}
  };
  
}

#endif // NPM_MANAGER_HPP
