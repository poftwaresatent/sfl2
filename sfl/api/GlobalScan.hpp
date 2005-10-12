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


#ifndef SUNFLOWER_GLOBALSCAN_HPP
#define SUNFLOWER_GLOBALSCAN_HPP


#include <sfl/api/Scan.hpp>
#include <sfl/util/Frame.hpp>
#include <boost/shared_ptr.hpp>


namespace sfl {


  /**
     Just like Scan, but also contains the scan points in the global
     frame of reference.
  */
  class GlobalScan
  {
  public:
    class data_t {
    public:
      /** x-coordinates [m] in the global frame */      
      double globx;
      /** y-coordinates [m] in the global frame */      
      double globy;
    };
    
    typedef std::vector<data_t> array_t;
    
    
    array_t data;
    const boost::shared_ptr<const Scan> local_scan;
    const Frame robot_position;
    
    
    /**
       Constructs a GlobalScan instance from an existing Scan
       instance, given the position of the robot at the time when the
       scan was taken.
       
       \pre valid local_scan (pay attention when using
       Scanner::GetScanCopy(), which can return 0 pointers on
       acquisition errors!)
    */
    GlobalScan(/** local scan object to extend */
	       boost::shared_ptr<const Scan> local_scan,
	       /** robot position, for calculating global points */
	       const Frame & position);
  };
  
}

#endif // SUNFLOWER_GLOBALSCAN_HPP
