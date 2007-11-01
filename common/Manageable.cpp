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


#include "Manageable.hpp"
#include "Manager.hpp"
#include <iostream>


using namespace std;


namespace npm {
  
  
  Manageable::
  Manageable(const string & _name,
	     const string & _comment,
	     boost::shared_ptr<Manager> manager)
    : name(_name),
      comment(_comment),
      m_manager(manager)
  {
    if (manager)
      manager->Attach(this);
  }
  
  
  Manageable::
  ~Manageable()
  {
    if (m_manager)
      m_manager->Detach(this);
  }
  
  
  ostream & Manageable::
  Print(ostream & os) const
  {
    os << name;
    if ( ! m_manager)
      os << " (non-managed)";
    if ( ! comment.empty())
      os << ": " << comment;
    return os;
  }
  
  
  ostream & operator << (ostream & os, const Manageable & m)
  {
    return m.Print(os);
  }
  
  
}
