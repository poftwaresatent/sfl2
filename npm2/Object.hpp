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
#include <set>


namespace npm2 {
  
  using namespace sfl;
  using namespace std;
  
  
  class Sensor;
  
  class Object
  {
  public:
    Object ();
    virtual ~Object();
    
    void setParent (Object * obj);
    
    /* \note Assumes parent has been updated, and recurses into all
       children. */
    void updateTransform ();
    
    void updateSensor (Sensor * sensor);
    
    Frame const & getGlobal () const { return global_; }
    
    Frame mount_;
    Frame motion_;
    Body body_;
    
  protected:    
    typedef set <Object*> children_t;
    
    Frame global_;
    Object * parent_;
    children_t children_;
  };
  
}

#endif // NPM2_OBJECT_HPP
