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


#ifndef NPM_MANAGEABLE_HPP
#define NPM_MANAGEABLE_HPP


#include <boost/shared_ptr.hpp>
#include <string>
#include <iosfwd>


namespace npm {
  
  class Manager;
  
  class Manageable
  {
  public:
    /**
       If you specify a non-null Manager, a Manageable object will
       automatically registers itself with it, and will deregister
       upon destruction.
    */
    Manageable(/** name of this instance, should be unique for the
		   given manager (if any) */
	       const std::string & name,
	       /** displayed as help-text to users */
	       const std::string & comment,
	       /** automatically register and deregister with this
		   manager (optional but recommended) */
	       boost::shared_ptr<Manager> manager);
    
    virtual ~Manageable();
    
    virtual std::ostream & Print(std::ostream & os) const;
    
    friend
    std::ostream & operator << (std::ostream & os, const Manageable & m);
    
    const std::string name;
    const std::string comment;

  protected:
    boost::shared_ptr<Manager> const m_manager;
  };
  
}

#endif // NPM_MANAGEABLE_HPP
