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

	/**
		 \todo Take data size from base class instead of computing them
		 again, in order to avoid future bugs when maybe changing the
		 formula in one or the other implementation.
	*/
	TraversabilityMapCS::
	TraversabilityMapCS(const Frame &origin, 
											shared_ptr<Sprite> sprite, 
											double resolution, double xSize, double ySize)
		: TraversabilityMap(origin, resolution, xSize,  ySize),
			references_(new array2d<Sprite::indexlist_t> (static_cast<size_t>(rint(xSize/resolution)),
																										static_cast<size_t>(rint(ySize/resolution))) ),
			sprite_(sprite)
	{
	}

	bool TraversabilityMapCS::
	SetValue(double global_x, double global_y, int & value)
	{
		cout << "Todo: SetValue not defined yet for Configuration Space Maps." <<endl
				 << "For now use SetFree(x, y) and SetObst(x, y)" << endl; 
		return true;
	}


	bool TraversabilityMapCS::
	SetObst(double global_x, double global_y)
	{
		//cout << "  ***  " <<endl;
		//cout << "Set obstacle at " << global_x << " " << global_y << endl;
	
		gframe.From(global_x, global_y);

		//cout << "After gframe.From "  << global_x << " " << global_y << endl;

		Region *region = new Region(sprite_, global_x, global_y, data->xsize, data->ysize);
		Sprite::indexlist_t csMask = region->GetArea();

		const GridFrame::index_t idx(gframe.GlobalIndex(global_x, global_y));

		for (unsigned int i(0); i<csMask.size(); i++)
			{
			
				// update references
				Sprite::sindex ref(idx.v0, idx.v1 , 1.0 - csMask.at(i).r);
	
				GridFrame::index_t csidx;
				csidx.v0 = csMask.at(i).x;
				csidx.v1 = csMask.at(i).y;
			
				//cout << " affects grid cell " << csMask.at(i).x << " " << csMask.at(i).y << endl;

				if (references_->ValidIndex(csidx))
					(*references_)[csidx].push_back(ref);
	
				// update map
				if(data->ValidIndex(csidx))
					(*data)[csidx] = obstacle;//doubleToInt(1.0 - csMask.at(i).r);
			}
		return true;
	}

	bool TraversabilityMapCS::
	SetFree(double global_x, double global_y)
	{
		gframe.From(global_x, global_y);
	
		Region *region = new Region(sprite_, global_x, global_y, data->xsize, data->ysize);
		Sprite::indexlist_t csMask = region->GetArea();

		const GridFrame::index_t idx(gframe.GlobalIndex(global_x, global_y));

		for (size_t i(0); i<csMask.size(); i++)
			{
				GridFrame::index_t csidx;
				csidx.v0 = csMask.at(i).x;
				csidx.v1 = csMask.at(i).y;

				for (size_t j(0);j<((*references_)[csidx].size());j++)
					{
						// delete reference to parent cell
						if (((*references_)[csidx][j].x == static_cast<ssize_t>(idx.v0))
								&& ((*references_)[csidx][j].y == static_cast<ssize_t>(idx.v1)))
							(*references_)[csidx].erase((*references_)[csidx].begin()+j);
					}

				double max(0.0);

				for (size_t j(0);j<((*references_)[csidx].size());j++)
					{
						//get remaining max value 
						if (max < ((*references_)[csidx][j].r)) max = ((*references_)[csidx][j].r); 
					}			

				// update map
				if(data->ValidIndex(csidx))
					(*data)[csidx] = doubleToInt(max);
			}
	
		return true;
	}

	inline int TraversabilityMapCS::
	doubleToInt(const double &val)
	{
		return static_cast<int>(rint((obstacle-freespace)*val));
	}

}
