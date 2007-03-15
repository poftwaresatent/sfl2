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


#ifndef SUNFLOWER_TRAVERSABILITY_MAP_HPP
#define SUNFLOWER_TRAVERSABILITY_MAP_HPP


#include <sfl/util/array2d.hpp>
#include <sfl/gplan/GridFrame.hpp>
#include <sfl/util/Frame.hpp>
#include <boost/shared_ptr.hpp>
#include <string>
#include <iosfwd>


namespace sfl {
  
  
  class TraversabilityMap
  {
  public:
    TraversabilityMap();
		TraversabilityMap(const GridFrame & origin, size_t ncells_x, size_t ncells_y);
		TraversabilityMap(const GridFrame & origin, size_t ncells_x, size_t ncells_y,
											int _freespace, int _obstacle, const std::string & _name);
		
    virtual ~TraversabilityMap() { }
		
    static boost::shared_ptr<TraversabilityMap>
    Parse(std::istream & is, std::ostream * os);
    
    /**
       \return true if the traversability at the given global
       coordinates is known, in which case the out-parameter
       <code>value</code> is set to it.
    */
    virtual bool GetValue(double global_x, double global_y, int & value) const;
    virtual bool SetValue(double global_x, double global_y, int value,
													GridFrame::draw_callback * cb);
		
		virtual bool SetObst(double global_x, double global_y,
												 GridFrame::draw_callback * cb)
		{
			return SetValue(global_x, global_y, obstacle, cb);
		}

		virtual bool SetFree(double global_x, double global_y,
												 GridFrame::draw_callback * cb)
		{
			return SetValue(global_x, global_y, freespace, cb);
		}
		
		void DumpMap(std::ostream * os);
    
    GridFrame gframe;		/**< default (0, 0, 0, 1) */
    int freespace;		/**< default 0 */
    int obstacle;		/**< default 127 */
    std::string name;		/**< default "world" */
    boost::shared_ptr<array2d<int> >  data; /**< default NULL */
    int mindata;		/**< only valid after successful Parse() */
    int maxdata;		/**< only valid after successful Parse() */
  };
  
}

#endif // SUNFLOWER_TRAVERSABILITY_MAP_HPP
