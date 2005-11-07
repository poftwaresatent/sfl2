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


#ifndef SUNFLOWER_MULTISCANNER_HPP
#define SUNFLOWER_MULTISCANNER_HPP


#include <sfl/api/Scanner.hpp>
#include <sfl/api/GlobalScan.hpp>
#include <boost/shared_ptr.hpp>


namespace sfl {


  /**
     Manages several scanners that can be mounted on the robot,
     provides a little unifying functionality.

     To use Multiscanner, first create instances for all the scanners
     on your robot. Then add them to a Multiscanner instance (in the
     order with which you'd like the data to appear in collected
     scans).
  */
  class Multiscanner
  {
  public:
    /** Appends a Scanner instance to the list of registered devices. */
    void Add(boost::shared_ptr<Scanner> scanner);

    /** \return The number of registered Scanners. */
    index_t Nscanners() const;

    /** \return A pointer to a registered Scanner, or 0 if no such index. */
    boost::shared_ptr<Scanner> GetScanner(index_t i) const;

    /**
       Concatenates data from all Scanners into a single Scan
       object. Only successfully retreived data is added to the
       returned Scan instance, so Scan::Nscans() might return a number
       smaller than what you would expect from the sum of
       Scanner::Nscans().
       
       Timestamps of the returned Scan correspond to the minimum and
       maximum Timestamp of all registered Scanners. Even if a Scanner
       provides no data (e.g. all values are out of range), its
       Timestamp is still taken into account.
       
       \note For polar coordinates, the robot origin is used. The
       ordering of the data is inherited from the order of calls to
       Multiscanner::Add(). Thus it is not guaranteed that the angles
       are monotonically increasing!
    */
    boost::shared_ptr<Scan> CollectScans() const;
    
    /**
       Like CollectScans() but also transforms the scan to global
       coordinates and returns a global scan object.
    */
    boost::shared_ptr<GlobalScan>
    CollectGlobalScans(const Frame & position) const;
    
    /** Mainly for debugging, returns the offset of a scanner's data
	in the collected scan object. This is usually not needed and
	determined implicitly inside CollectScans() and
	CollectGlobalScans().*/
    size_t ComputeOffset(boost::shared_ptr<const Scanner> scanner) const;
    
    /** \note A bit of a hack for Cogniron: Call Update() on all
	registered Scanner instances. */
    int UpdateAll();
    
    
  protected:
    typedef std::vector<boost::shared_ptr<Scanner> > vector_t;
    
    index_t m_total_nscans;
    vector_t m_scanner;
  };
  
}

#endif // SUNFLOWER_MULTISCANNER_HPP
