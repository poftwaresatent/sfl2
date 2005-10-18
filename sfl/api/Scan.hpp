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


#ifndef SUNFLOWER_SCAN_HPP
#define SUNFLOWER_SCAN_HPP


#include <sfl/api/Timestamp.hpp>
#include <sfl/api/types.hpp>
#include <vector>


namespace sfl {


  /**
     Encapsulates the whole dataset of a scan, useful for algorithms
     that are formulated for using arrays of data. Contains timestamp
     information in the form of an upper and a lower bound, this makes
     it useful in settings where scans are collected from multiple
     sensors.
  */
  class Scan
  {
  public:
    class data_t {
    public:
      /** ray angles [rad] */      
      double phi;
      /** measured distances [m] */      
      double rho;
      /** x-coordinates [m] in the robot frame */      
      double locx;
      /** y-coordinates [m] in the robot frame */      
      double locy;
    };
    
    
    /**
       Constructor for empty (zeroed arrays) data sets. By default,
       the acquisition time bounds Scan::_tlower and Scan::_tupper are
       initialized to the minimum (Timestamp::First()) and maximum
       (Timestamp::Last()) of all possible time values.
    */
    Scan(/** number of data points */
	 size_t nscans,
	 /** lower bound on the acquisition timestamp */
	 const Timestamp & tlower = Timestamp::First(),
	 /** upper bound on the acquisition timestamp */
	 const Timestamp & tupper = Timestamp::Last());
    
    Scan(const Scan & original);
    
    
    size_t GetNScans() const { return m_data.size(); }
    const Timestamp & GetTLower() const { return m_tlower; }
    const Timestamp & GetTUpper() const { return m_tupper; }
    
    /** \pre index < GetNScans() */
    const data_t & GetData(size_t index) const { return m_data[index]; }
    
    
  protected:
    friend class Scanner;
    friend class Multiscanner;
    
    typedef std::vector<data_t> array_t;
    
    /** lower bound of the estimated acquisition time */
    Timestamp m_tlower;
    
    /** upper bound of the estimated acquisition time */
    Timestamp m_tupper;
    
    /** array of scan data */
    array_t m_data;
  };
  
}

#endif // SUNFLOWER_SCAN_HPP
