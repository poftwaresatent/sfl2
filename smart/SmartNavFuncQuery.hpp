/* 
 * Copyright (C) 2006
 * Swiss Federal Institute of Technology, Zurich. All rights reserved.
 * 
 * Developed at the Autonomous Systems Lab <http://www.asl.ethz.ch/>
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


#ifndef NPM_SMART_NAVFUNCQUERY_HPP
#define NPM_SMART_NAVFUNCQUERY_HPP


#include <asl/path_tracking.hpp>
#include <boost/scoped_ptr.hpp>
#include <vector>


class Smart;

namespace asl {
  struct path_element;
  typedef std::vector<path_element> path_t;
}

namespace estar {
  class Facade;
  class Region;
  struct carrot_item;
  typedef std::vector<carrot_item> carrot_trace;
}

namespace sfl{
  class Goal;
}

class SmartNavFuncQuery
  : public asl::NavFuncQuery
{
public:
  SmartNavFuncQuery(Smart * smart);
  virtual status_t GetValue(double globalx, double globaly,
			    double & value) const;
  virtual status_t ComputeDeltaCost(double x0, double y0, double x1, double y1,
				    double & delta) const;
  virtual status_t GetPath(double lookahead, double stepsize, double globalx, double globaly, asl::path_t & path) const;

  virtual status_t IsWithinBounds(double globalx, double globaly) const;
private:
  Smart * m_smart;
};

#endif // NPM_SMART_NAVFUNCQUERY_HPP
