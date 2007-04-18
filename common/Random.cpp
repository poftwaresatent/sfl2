/* 
 * Copyright (C) 2005 Roland Philippsen <roland dot philippsen at gmx dot net>
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


#include "Random.hpp"
#include <iostream>
#include <sfl/util/numeric.hpp>
#include <fstream>


using namespace std;
using namespace sfl;


Random::
Random():
  _is(new ifstream("/dev/urandom"))
{
}


bool Random::
Uniform(double chance)
{
  return This()->Roll() <= chance * numeric_limits<uint32_t>::max();
}


uint32_t Random::
Roll()
{
  uint32_t result(0);
  for(int i(0); i < 4; ++i){
    char byte;
    if( ! ((*_is) >> byte)){
      cerr << "ABORT in Random::Roll()\n";
      exit(EXIT_FAILURE);
    }
    result |= (byte & 0xFF) << (i * 8);
  }
  //  printf("roll: 0x%04X\n", result);
  return result;
}


Random * Random::
This()
{
  static auto_ptr<Random> that;
  if(that.get() == 0)
    that = auto_ptr<Random>(new Random());
  return that.get();
}


double Random::
Unit()
{
  return double(This()->Roll()) / numeric_limits<uint32_t>::max();
}


double Random::
Uniform(double vmin, double vmax)
{
  const double unit(Unit());
  return minval(maxval(unit * vmin + (1 - unit) * vmax, vmin), vmax);
}


int Random::
Uniform(int vmin, int vmax)
{
  while(true){
    const int v(static_cast<int>(floor(Uniform(static_cast<double>(vmin),
					       static_cast<double>(vmax+1)))));
    if(v <= vmax)		// tiny chance of getting vmax + 1
      return v;
  }
}
