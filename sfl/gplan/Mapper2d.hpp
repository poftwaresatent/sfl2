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


#ifndef SFL_MAPPER2D_HPP
#define SFL_MAPPER2D_HPP


#include <sfl/util/Frame.hpp>


namespace sfl {

	
	class Scanner;
	class TraversabilityMap;

  
  class Mapper2d{
  public:
		Mapper2d();
		
		/**
			 Update of traversability Map based on Scan.

			 \todo better to use a Scan object, then it doesn't matter if it
			 comes from one or several Scanner instances (e.g. use
			 Multiscanner::CollectScans().
		*/
		bool update(const Frame &odo, const Scanner &scanner);
		
		boost::shared_ptr<sfl::TraversabilityMap> getTravMap();
		
	private:
		bool simpleCellUpdate(double x, double y);
		bool swipeCellUpdate(double x, double y);
		int map_obstacle_between_trace(int fx0, int fy0, int fx1, int fy1);
		
		boost::shared_ptr <sfl::TraversabilityMap> travMap_;
		sfl::Frame odo_;
	};

}

#endif // SFL_MAPPER2D_HPP
