/* Nepumuk Mobile Robot Simulator v2
 *
 * Copyright (C) 2014 Roland Philippsen. All rights reserved.
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

#ifndef NPM2_OBJECT_HPP
#define NPM2_OBJECT_HPP

#include <npm2/Body.hpp>
#include <sfl/util/Frame.hpp>
#include <fpplib/configurable.hpp>
#include <set>
#include <string>


namespace npm2 {
  
  using namespace sfl;
  using namespace std;
  
  
  class Sensor;
  
  class Object
    : public fpplib::Configurable
  {
  public:
    typedef fpplib::Registry<Object, false> registry_t;
    static registry_t registry;
    
    typedef set <Object*> children_t;
    typedef children_t::const_iterator child_iterator_t;
    
    explicit Object (string const & name);
    virtual ~Object();
    
    /** \note Does not perform sanity checks (i.e.: if you pass obj to
	its own setParent() you create a problem...) */
    void setParent (Object * obj);
    
    /** Configurability method. The parent object is looked up in the
	registry, and this method fails (returns false) if you attempt
	to set itself as parent or the name is not found in the
	registry. */
    bool findSetParent (string const & name);
    
    /** Configurability method.  Adds a line to the body_ of this
	object. */
    bool addLine (Line const & line);
    
    /* \note Assumes parent has been updated, and recurses into all
       children. */
    void updateTransform ();
    
    void updateSensor (Sensor * sensor);
    
    Frame const & getGlobal () const { return global_; }
    
    /** \note The bounds include the bodies of all children. */
    BBox const & getBBox () const { return bbox_; }
    
    child_iterator_t const childBegin () const { return children_.begin(); }
    child_iterator_t const childEnd () const { return children_.end(); }

    Frame mount_;
    Frame motion_;
    Body body_;
    
  protected:    
    Frame global_;
    Object * parent_;
    children_t children_;
    BBox bbox_;
  };
  
}

#endif // NPM2_OBJECT_HPP
