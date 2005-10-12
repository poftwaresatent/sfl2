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


#include "Timestamp.hpp"
#include "HALProxy.hpp"
#include <sstream>
#include <limits>
#include <iomanip>


using namespace std;


namespace sfl {
  
  
  int Timestamp::
  Now(HALProxy & halProxy,
      struct timespec * spec)
  {
    return halProxy.hal_time_get( * spec);
  }
  
  
  const Timestamp & Timestamp::
  Last()
  {
    static const struct timespec spec = {
      tv_sec:  numeric_limits<long>::max(),
      tv_nsec: numeric_limits<long>::max() };
    static const Timestamp last(spec);
    
    return last;
  }
  
  
  const Timestamp & Timestamp::
  First()
  {
    static const struct timespec spec = {
      tv_sec:  numeric_limits<long>::min(),
      tv_nsec: numeric_limits<long>::min() };
    static const Timestamp first(spec);
    
    return first;
  }
  
  
  ostream & operator << (ostream & os,
			 const Timestamp & t)
  {
    char oldfill(os.fill('0'));
    os << t.m_stamp.tv_sec << "." << setw(9) << t.m_stamp.tv_nsec;
    os.fill(oldfill);
    return os;
  }
  
}
