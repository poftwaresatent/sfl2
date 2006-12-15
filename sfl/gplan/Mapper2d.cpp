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


#include <sfl/api/Scan.hpp>
#include <estar/Sprite.hpp>
#include <iostream>

#include "Mapper2d.hpp"
#include "TraversabilityMapCS.hpp"


using namespace std;
using namespace estar;
using namespace boost;


namespace sfl {
	
	
	/**
		 \todo get stuff as parameters (from config file) instead of magic
		 numbers
	*/ 
	Mapper2d::Mapper2d()
		: travMap_(new TraversabilityMapCS(Frame(-10.0 ,-60.0 ,0),
																			 shared_ptr<Sprite>(new Sprite(1.0, 0.5)),
																			 0.5, 100.0, 100.0))
		
	{
	}
	
	
	shared_ptr<TraversabilityMap> Mapper2d::getTravMap()
	{
		return travMap_;
	}
	
	
	bool Mapper2d::update(const Frame &odo, const Scan &scan,
												GridFrame::draw_callback * cb)
	{
		const Scan::array_t & scan_data(scan.data);
		for (size_t i(0); i< scan_data.size(); i++)
				simpleCellUpdate(scan_data[i].globx, scan_data[i].globy, cb);

		odo_ = odo;

		return true;
	}

	// Private functions

	inline bool Mapper2d::simpleCellUpdate(double x, double y,
																				 GridFrame::draw_callback * cb)
	{
		travMap_->SetObst(x, y, cb);
		return true; 
	}

	bool Mapper2d::swipeCellUpdate(double x, double y,
																 GridFrame::draw_callback * cb)
	{
	
		double x0(odo_.X());
		double y0(odo_.Y());

		const GridFrame::index_t idx0(travMap_->gframe.GlobalIndex(x0, y0));
		const GridFrame::index_t idx(travMap_->gframe.GlobalIndex(x, y));

		map_obstacle_between_trace(idx0.v0, idx0.v1, idx.v0, idx.v1);

		travMap_->SetObst(x, y, cb);
	
		return true;
	}

  /* run Bresenham's line drawing algorithm, adapted from:
		 www.cs.nps.navy.mil/people/faculty/capps/iap/class1/lines/lines.html 
	*/
	int Mapper2d::map_obstacle_between_trace(int fx0, int fy0, int fx1, int fy1){

		int x0 = fx0, y0 = fy0, x1 = fx1, y1 = fy1;

		GridFrame::index_t idx(travMap_->gframe.GlobalIndex(x0, y0));

		int dy = y1 - y0;
		int dx = x1 - x0;
		int stepx, stepy;
		int fraction;

		if (dy < 0) { dy = -dy;  stepy = -1; } else { stepy = 1; }
		if (dx < 0) { dx = -dx;  stepx = -1; } else { stepx = 1; }
		dy <<= 1;                            /* dy is now 2*dy*/
		dx <<= 1;                            /* dx is now 2*dx*/

		idx.v0 = x0;
		idx.v1 = y0;
		if(travMap_->data->ValidIndex(idx))
			if ((*travMap_->data)[idx] == travMap_->obstacle)
				{
					(*travMap_->data)[idx] = travMap_->freespace;
				}

		if (dx > dy) {
			fraction = dy - (dx >> 1);                 /* same as 2*dy - dx */
			while (x0 != x1) {
				if (fraction >= 0) {
					y0 += stepy;
					fraction -= dx;                 /* same as fraction -= 2*dx */
				}
				x0 += stepx;
				fraction += dy;                   /* same as fraction -= 2*dy */
			 
				idx.v0 = x0;
				idx.v1 = y0;
				if(travMap_->data->ValidIndex(idx))
					if ((*travMap_->data)[idx] == travMap_->obstacle)
						{
							(*travMap_->data)[idx] = travMap_->freespace;
						}
			}
			idx.v0 = x0;
			idx.v1 = y0;
			if(travMap_->data->ValidIndex(idx))
				if ((*travMap_->data)[idx] == travMap_->obstacle)
					{
						(*travMap_->data)[idx] = travMap_->freespace;
					}
		} 
		else {
			fraction = dx - (dy >> 1);
			while (y0 != y1) {
				if (fraction >= 0) {
					x0 += stepx;    
					fraction -= dy;
				}
				y0 += stepy;
				fraction += dx;
			 
				idx.v0 = x0;
				idx.v1 = y0;
				if(travMap_->data->ValidIndex(idx))
					if ((*travMap_->data)[idx] == travMap_->obstacle)
						{
							(*travMap_->data)[idx] = travMap_->freespace;
						}
		 
			}
		 
			idx.v0 = x0;
			idx.v1 = y0;
			if(travMap_->data->ValidIndex(idx))
				if ((*travMap_->data)[idx] == travMap_->obstacle)
					{
						(*travMap_->data)[idx] = travMap_->freespace;
					}

		}
		return 0;
	}

}
