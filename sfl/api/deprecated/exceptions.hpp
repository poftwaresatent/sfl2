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


#ifndef SUNFLOWER_EXCEPTIONS_HPP
#define SUNFLOWER_EXCEPTIONS_HPP


#include <stdexcept>


namespace sfl {


  /**
     Exception thrown by entities that need to interact with the
     HAL. This helps keeping only those places HAL-dependant that can
     actually know about the HAL's functionality.
  */
  class hal_error:
    public std::runtime_error
  {
  public:
    hal_error(const std::string & why,
	      int retval):
      runtime_error(why),
      _retval(retval) { }

    int retval() const { return _retval; }

  protected:
    int _retval;
  };


  /**
     Exception thrown by entities with array-semantics (provided they
     perform bound checks).
  */
  template<typename _index_t>
  class index_error:
    public std::runtime_error
  {
  public:
    index_error(const std::string & why,
		_index_t index):
      runtime_error(why),
      _index(index) { }

    _index_t index() const { return _index; }
 
  protected:
    _index_t _index;
  };


  /**
     Only to be used for new developments when you want to program
     clients to an inexisting interface implementation. Instead of
     silently doing nothing, it is often preferrable to throw this
     exception for making sure that all parts of a class interface
     have been implemented.
  */
  class not_implemented:
    public std::runtime_error
  {
  public:
    not_implemented(const std::string & why):
      std::runtime_error(why) { }
  };

}

#endif // SUNFLOWER_EXCEPTIONS_HPP
