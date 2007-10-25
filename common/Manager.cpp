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


#include "Manager.hpp"
#include "Manageable.hpp"
#include <iostream>


using namespace std;


namespace npm {


  Manageable * Manager::
  Retrieve(const string & name) const
  {
    catalog_t::const_iterator ic(catalog.find(name));
    if(catalog.end() == ic)
      return 0;
    return ic->second;
  }


  Manageable * Manager::
  Attach(Manageable * entry)
  {
    if(entry->name == "")
      return 0;
    catalog_t::iterator ic(catalog.find(entry->name));
    if(catalog.end() == ic){
      catalog.insert(make_pair(entry->name, entry));
      return 0;
    }
    Manageable * oldentry(ic->second);
    ic->second = entry;
    return oldentry;
  }
  
  
  bool Manager::
  Detach(const Manageable * entry)
  {
    if(entry->name == "")
      return true;
    catalog_t::iterator ic(catalog.find(entry->name));
    if(catalog.end() == ic)
      return false;
    Manageable * oldentry(ic->second);
    catalog.erase(ic);
    return entry == oldentry;
  }
  
  
  void Manager::
  PrintCatalog(ostream & os) const
  {
    for(catalog_t::const_iterator ic(catalog.begin());
	ic != catalog.end(); ++ic)
      os << ic->second->name << "\n";
  }
  
}
