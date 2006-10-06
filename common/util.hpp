/* 
 * Copyright (C) 2006 Roland Philippsen <roland dot philippsen at gmx dot net>
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


#ifndef NPM_UTIL_HPP
#define NPM_UTIL_HPP


#include <string>
#include <sstream>


namespace npm {
  
  /** convert "anything" to a string by using its output operator */
  template<typename Foo>
  std::string to_string(const Foo & foo) {
    std::ostringstream os;
    os << foo;
    return os.str();
  }
  
  /** convert a string to "something" based on its input operator */
  template<typename Foo>
  bool string_to(const std::string & str, Foo & foo) {
    Foo bar;
    std::istringstream is(str);
    if( ! (is >> bar))
      return false;
    foo = bar;
    return true;
  }
  
  /** booleans are converted by string matching: "true", "True",
      "TRUE", "on", "On", or "ON" yield a true boolean, whereas
      "false", "False", "FALSE", "off", "Off", or "OFF" yield a false
      boolean. Anything else results in a failure (and foo is not
      touched). */
  template<>
  bool string_to<bool>(const std::string & str, bool & foo);
  
  /** very useful for booleans that are encoded as char, int, short,
      ... sets them to 1 if string_to<bool> yields true, or to 0 if
      string_to<bool> yields false, but doesn't touch foo if
      string_to<bool> failed. */
  template<typename Foo>
  bool string_to_bool(const std::string & str, Foo & foo) {
    bool bar;
    if( ! string_to(str, bar))
      return false;
    foo = bar ? 1 : 0;
    return true;
  }
  
}

#endif // NPM_UTIL_HPP
