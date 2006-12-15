/* -*- mode: C++; tab-width: 2 -*- */
/* 
 * Copyright (C) 2006
 * Swiss Federal Institute of Technology, Zurich. All rights reserved.
 * 
 * Developed at the Autonomous Systems Lab.
 * Visit our homepage at http://www.asl.ethz.ch/
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


#include <iostream>
#include <math.h>
#include <sfl/gplan/GridFrame.hpp>
#include "TraversabilityMapCS.hpp"

using namespace std;
using namespace estar;
using namespace boost;

namespace sfl {

	TraversabilityMapCS::
	TraversabilityMapCS(const Frame &origin, 
											shared_ptr<Sprite> sprite, 
											double resolution, double xSize, double ySize)
		: TraversabilityMap(origin, resolution, xSize,  ySize),
			references_(new array2d<refmap_t>(data->xsize, data->ysize)),
			sprite_(sprite)
	{
	}
	
	
	bool TraversabilityMapCS::
	SetValue(double global_x, double global_y, int value,
					 GridFrame::draw_callback * cb)
	{
		cout << "Todo: SetValue not defined yet for Configuration Space Maps.\n"
				 << "For now use SetFree(x, y) and SetObst(x, y)\n"; 
		return TraversabilityMap::SetValue(global_x, global_y, value, cb);
	}
	
	
	bool TraversabilityMapCS::
	SetObst(double global_x, double global_y,
					GridFrame::draw_callback * cb)
	{
		gframe.From(global_x, global_y);
		Region region(sprite_, global_x, global_y, data->xsize, data->ysize);
		const Sprite::indexlist_t & csMask(region.GetArea());
		const GridFrame::index_t idx(gframe.GlobalIndex(global_x, global_y));
		for (size_t ii(0); ii < csMask.size(); ++ii)
			{
				const GridFrame::index_t csidx(csMask[ii].x, csMask[ii].y);				
				if(references_->ValidIndex(csidx)){
					// update references
					(*references_)[csidx].insert(make_pair(1.0 - csMask[ii].r, idx));
					// update map
					(*data)[csidx] = obstacle;//doubleToInt(1.0 - csMask.at(i).r);
					// notify callback
					if(cb)
						(*cb)(csidx.v0, csidx.v1);
				}
			}
		return true;
	}
	
	
	bool TraversabilityMapCS::
	SetFree(double global_x, double global_y,
					GridFrame::draw_callback * cb)
	{
		gframe.From(global_x, global_y);
		
		Region region(sprite_, global_x, global_y, data->xsize, data->ysize);
		const Sprite::indexlist_t & csMask(region.GetArea());
		const GridFrame::index_t idx(gframe.GlobalIndex(global_x, global_y));
		for (size_t ii(0); ii < csMask.size(); ++ii)
			{
				GridFrame::index_t csidx(csMask[ii].x, csMask[ii].y);
				if(references_->ValidIndex(csidx)){
					refmap_t & ref((*references_)[csidx]);
					vector<refmap_t::iterator> tbd;
					for(refmap_t::iterator jj(ref.begin()); jj != ref.end(); ++jj)
						if (jj->second == idx)
							tbd.push_back(jj);
					for(size_t jj(0); jj < tbd.size(); ++jj)
						ref.erase(tbd[jj]);
					if(ref.empty())
						(*data)[csidx] = freespace;
					else
						(*data)[csidx] = doubleToInt(ref.begin()->first);
					if(cb)
						(*cb)(csidx.v0, csidx.v1);
				}
			}
		
		return true;
	}
	
	
	inline int TraversabilityMapCS::
	doubleToInt(const double &val)
	{
		return static_cast<int>(rint((obstacle-freespace)*val));
	}

}
