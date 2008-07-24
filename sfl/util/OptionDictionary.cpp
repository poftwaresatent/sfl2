/* 
 * Copyright (C) 2005
 * Centre National de Recherche Scientifique, France.
 * All rights reserved.
 * 
 * Developed at
 * Laboratoire d'Automatique et d'Analyse des Systemes, LAAS-CNRS.
 * Visit our homepage at http://www.laas.fr/
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


#include "OptionDictionary.hpp"

using namespace std;

namespace sfl {


  string OptionDictionary::
  GetOption(const string & key) const
  {
    option_t::const_iterator io(m_option.find(key));
    if(io == m_option.end())
      return string("");
    return io->second;
  }


  void OptionDictionary::
  SetOption(const string & key, const string & value)
  {
    option_t::iterator io(m_option.find(key));
    if(io == m_option.end())
      m_option.insert(make_pair(key, value));
    else
      io->second = value;
  }

}
