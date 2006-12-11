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
		TraversabilityMap(Frame &origin, double resolution, double xSize, double ySize); 
    
    static boost::shared_ptr<TraversabilityMap>
    Parse(std::istream & is, std::ostream * os);
    
    /**
       \return true if the traversability at the given global
       coordinates is known, in which case the out-parameter
       <code>value</code> is set to it.
    */
    bool GetValue(double global_x, double global_y, int & value) const;
    bool SetValue(double global_x, double global_y, int & value);

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
