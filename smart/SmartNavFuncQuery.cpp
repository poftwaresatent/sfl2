
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
#include <sfl/util/Frame.hpp>
#include <sfl/api/Goal.hpp>
#include <estar/Facade.hpp>
#include <estar/util.hpp>

#ifdef NPM_DEBUG
# define PDEBUG PDEBUG_OUT
#else // ! NPM_DEBUG
# define PDEBUG PDEBUG_OFF
#endif // NPM_DEBUG
#define PVDEBUG PDEBUG_OFF

#define TEMP_MAX_META 100000000

using namespace sfl;
using namespace asl;
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
  if(( ! m_smart->m_travmap)
     || ( ! m_smart->m_travmap->data)
     || ( ! m_smart->m_estar))
    return INVALID;
  
  const GridFrame::index_t
    idx(m_smart->m_travmap->gframe.GlobalIndex(globalx, globaly));
  shared_ptr<const array2d<int> > data(m_smart->m_travmap->data);
  
  if((idx.v0 >= data->xsize) || (idx.v1 >= data->ysize))
    return OUT_OF_BOUNDS;
  
  value = m_smart->m_estar->GetValue(idx.v0, idx.v1);
  if(absval(value - m_smart->m_estar->GetObstacleMeta()) < 100 * epsilon)
    return OBSTACLE;
  return SUCCESS;
}

/* computes cost along a path segment */
SmartNavFuncQuery::status_t SmartNavFuncQuery::
ComputeDeltaCost(double x0, double y0, double x1, double y1,
	 double & delta) const
{

  SmartNavFuncQuery::status_t stat_t;

  if((stat_t=IsWithinBounds(x0,y0))!=SUCCESS){
    PDEBUG_OUT("x0, y0 out of bounds!\n");
    return stat_t;
  }
  if((stat_t=IsWithinBounds(x1,y1))!=SUCCESS){
    PDEBUG_OUT("x1, y1 out of bounds!\n");
    return stat_t;
  }

  const GridFrame::index_t
    idx_0(m_smart->m_travmap->gframe.GlobalIndex(x0, y0));
  const GridFrame::index_t
    idx_1(m_smart->m_travmap->gframe.GlobalIndex(x1, y1));

  double value_0 = m_smart->m_estar->GetValue(idx_0.v0, idx_0.v1);
  double value_1 = m_smart->m_estar->GetValue(idx_1.v0, idx_1.v1);


  /*current implementation assumes the return OBSTACLE if one of the grid nodes 
    is occupied (no interpolation then) */
  if(absval(value_0)> TEMP_MAX_META){
    return OBSTACLE;
  }

  if(absval(value_1)> TEMP_MAX_META){
    return OBSTACLE;
  }

  /* assuming linearly interpolated velocity between the two grid nodes */
  delta=(value_0-value_1)/2.0;

  return SUCCESS;
}

/* getting the global path from the navigation function a lookahead from
   the current robot position */
SmartNavFuncQuery::status_t SmartNavFuncQuery::
GetPath(double lookahead, double stepsize, double globalx, double globaly,  asl::path_t & path) const
{
 
  if(( ! m_smart->m_travmap)
     || ( ! m_smart->m_travmap->data)
     || ( ! m_smart->m_estar))
    return INVALID;
  
  double robx_grid(globalx);
  double roby_grid(globaly);
  
  /* no carrot_trace initialization assumed that it was initialized before in smart */
  if(!m_smart->m_carrot_trace){
    PVDEBUG("ERROR: The carrot path structure was not initialized!!!\n");
    return INVALID;
  }else{
    m_smart->m_carrot_trace->clear();
  }

  m_smart->m_travmap->gframe.From(robx_grid, roby_grid);

  const size_t maxnsteps(30);
  /*warning: the globalx, globaly coordinates are assumed to be already in the planning frame */
  const int result(trace_carrot(*(m_smart->m_estar),robx_grid, roby_grid,lookahead, stepsize, maxnsteps, *(m_smart->m_carrot_trace)));

  if(0 > result){
    PVDEBUG("FAILED compute_carrot()\n");
    return INVALID;
  }
  else{
    if(1 == result){ 
      PVDEBUG("WARNING: path didn't reach lookahead distance %g\n", lookahead);
    }
      if(m_smart->m_carrot_trace->back().value <= 3 * stepsize){
      PVDEBUG("carrot on goal border, appending goal point to carrot");
      double foox(m_smart->m_goal->X());
      double fooy(m_smart->m_goal->Y());
      m_smart->m_travmap->gframe.From(foox, fooy);
      m_smart->m_carrot_trace->push_back(estar::carrot_item(foox, fooy, 0, 0, 0, true));
      return INVALID;
    }
  }

  path.clear();

  for(size_t ii(0); ii < m_smart->m_carrot_trace->size(); ++ii){
    estar::carrot_item item((*(m_smart->m_carrot_trace))[ii]);
    m_smart->m_travmap->gframe.To(item.cx, item.cy);
    m_smart->m_travmap->gframe.RotateTo(item.gradx, item.grady);
    path.push_back(path_element(path_point(item.cx, item.cy),
				path_point(item.gradx, item.grady),
				item.value, item.degenerate));
   
  }
  return SUCCESS;
}


/* computes cost along a path segment */
SmartNavFuncQuery::status_t SmartNavFuncQuery::
IsWithinBounds(double x, double y) const
{


  if(( ! m_smart->m_travmap)
     || ( ! m_smart->m_travmap->data)
     || ( ! m_smart->m_estar))
    return INVALID;

  shared_ptr<const array2d<int> > data(m_smart->m_travmap->data);
  
  const GridFrame::index_t
    idx_0(m_smart->m_travmap->gframe.GlobalIndex(x, y));
 
  if((idx_0.v0 >= data->xsize) || (idx_0.v1 >= data->ysize))
    return OUT_OF_BOUNDS;
  
  return SUCCESS;

}
