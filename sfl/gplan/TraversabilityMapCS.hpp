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


#ifndef SFL_TRAVERSABILITY_MAP_CS
#define SFL_TRAVERSABILITY_MAP_CS

#include <estar/Sprite.hpp>
#include <estar/Region.hpp>

#include <sfl/gplan/TraversabilityMap.hpp>

#include <map>

namespace sfl{

	class TraversabilityMapCS
		: public TraversabilityMap
	{
	private:
		/** non-copyable */
		TraversabilityMapCS(const TraversabilityMapCS &);
		
	public:
		TraversabilityMapCS(const Frame &origin,
												boost::shared_ptr<estar::Sprite> sprite,
												double resolution, double xSize, double ySize); 

		virtual bool SetValue(double global_x, double global_y, int value,
													GridFrame::draw_callback * cb);
		virtual bool SetObst(double global_x, double global_y,
												 GridFrame::draw_callback * cb);
		virtual bool SetFree(double global_x, double global_y,
												 GridFrame::draw_callback * cb);
		
	private:
		typedef std::multimap<double, vec2d<size_t> > refmap_t;
		
		int doubleToInt(const double &val);
		
		boost::shared_ptr<array2d<refmap_t> >  references_;
		boost::shared_ptr<estar::Sprite> sprite_;
	};

	
}

#endif // SFL_TRAVERSABILITY_MAP_CS
