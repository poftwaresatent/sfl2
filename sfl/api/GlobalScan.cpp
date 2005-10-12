/* 
 * Copyright (C) 2004
 * Swiss Federal Institute of Technology, Lausanne. All rights reserved.
 * 
 * Developed at the Autonomous Systems Lab.
 * Visit our homepage at http://asl.epfl.ch/
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


#include "GlobalScan.hpp"


using boost::shared_ptr;


namespace sfl {
  
  
  GlobalScan::
  GlobalScan(shared_ptr<const Scan> _local_scan,
	     const Frame & position):
    data(_local_scan->data.size()),
    local_scan(_local_scan),
    robot_position(position)
  {
    for(size_t i(0); i < data.size(); ++i){
      data[i].globx = _local_scan->data[i].locx;
      data[i].globy = _local_scan->data[i].locy;
      robot_position.To(data[i].globx, data[i].globy);
    }
  }
  
}
