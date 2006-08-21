/* 
 * Copyright (C) 2006 Roland Philippsen <roland dot philippsen at gmx net>
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


#ifndef SUNFLOWER_FPRINTFOS_HPP
#define SUNFLOWER_FPRINTFOS_HPP


#include <sstream>
#include <stdio.h>


namespace sfl {
  
  
  /**
     Quickly hacked wrapper for printing to C-style FILE* using C++
     ostream semantics. Probably creates quite a bit of unnecessary
     runtime overhead.
  */
  class fprintfos
    : public std::ostream
  {
  public:
    fprintfos(FILE * cfile): std::ostream(0), m_cfile(cfile) {}
    
    template<typename foo>
    fprintfos & operator << (const foo & bar) {
      if(0 == m_cfile) return * this;
      std::ostringstream os;
      os << bar;
      fprintf(m_cfile, os.str().c_str());
      return * this;
    }
    
  private:
    FILE * m_cfile;
  };
  
}

#endif // SUNFLOWER_FPRINTFOS_HPP
