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

#include "SmartNavFuncQuery.hpp"
#include "Smart.hpp"
#include <sfl/gplan/TraversabilityMap.hpp>
#include <sfl/util/numeric.hpp>
#include <estar/Facade.hpp>

using namespace sfl;
using namespace boost;


SmartNavFuncQuery::
SmartNavFuncQuery(Smart * smart)
  : m_smart(smart)
{
}


SmartNavFuncQuery::status_t SmartNavFuncQuery::
GetValue(double globalx, double globaly,
	 double & value) const
{
  if(( ! m_smart->m_gframe)
     || ( ! m_smart->m_travmap)
     || ( ! m_smart->m_travmap->data)
     || ( ! m_smart->m_estar))
    return INVALID;
  
  const GridFrame::index_t
    idx(m_smart->m_gframe->GlobalIndex(globalx, globaly));
  shared_ptr<const array2d<int> > data(m_smart->m_travmap->data);
  
  if((idx.v0 >= data->xsize) || (idx.v1 >= data->ysize))
    return OUT_OF_BOUNDS;
  
  value = m_smart->m_estar->GetValue(idx.v0, idx.v1);
  if(absval(value - m_smart->m_estar->GetObstacleMeta()) < 100 * epsilon)
    return OBSTACLE;
  return SUCCESS;
}
