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


#ifndef SUNFLOWER_SCANNER_HPP
#define SUNFLOWER_SCANNER_HPP


#include <sfl/util/Frame.hpp>
#include <sfl/api/Scan.hpp>
#include <sfl/api/HAL.hpp>
#include <sfl/api/types.hpp>
#include <sfl/api/Timestamp.hpp>
#include <boost/shared_ptr.hpp>
#include <vector>


namespace sfl {
  
  
  /**
     Encapsulates a distance scanner such as the SICK-LMS. This class
     is generic in the sense that you have to provide it with a number
     of parameters to make it work correctly with the actual
     scanner(s) mounted on your robot:

     <ul>

       <li> Where the sensor is mounted on the robot. Scanner works in
            two coordinate frames: Polar coordinates (distance and ray
            angle) are represented in the sensor frame; Cartesian
            points are converted to the robot frame (using the
            provided <code>mount</code> information). </li>

       <li> Number of readings per scan (181 or 361 for SICK-LMS,
            depending on its configuration) </li>

       <li> Offset and sweep angles. This effectively defines the
            sensor frame: The first ray is considered to lie on
            <code>phi0</code> in the sensor frame, and the scanner
            sweeps a sector of <code>phirange</code> radians in the
            usual geometric sense (counterclockwise). If your sensor
            sweeps clockwise, just specify a negative
            <code>phirange</code>. </li>

       <li> Maximum valid distance (readings above this value are
            handled as <code>OUT_OF_RANGE</code>). </li>

     </ul>
     
     For example, the mount points, start and sweep angles on Robox are:
     
     <ul>
       <li> Front: <ul>
         <li> <code>mount</code>: (0.093, 0, 0) </li>
	 <li> <code>phi0</code>: - <code>M_PI</code> / 2 </li>
	 <li> <code>phirange</code>: <code>M_PI</code> </li> </ul> </li>
       <li> Rear: <ul>
         <li> <code>mount</code>: ( - 0.093, 0, <code>M_PI</code>) </li>
	 <li> <code>phi0</code>: - <code>M_PI</code> / 2 </li>
	 <li> <code>phirange</code>: <code>M_PI</code> </li> </ul> </li>
     </ul>
  */
  class Scanner
  {
  public:
    /**
       Return type of several accessors.
       
       Instead of using explicit <code>status_t</code> return values
       from the access methods, an exception mechanism might do the
       job "more elegantly". But <code>OUT_OF_RANGE</code> readings
       are actually quite common, not really an exception. It seemed
       inappropriate to write a <code>try ... catch</code> around all
       places where you access Scanner data.
       
       In most cases, you'll access scan data through a Scan instance
       anyways, which provides STL <code>vector</code> based semantics
       and is much more practical for handling a whole scan as a
       single object.
    */
    typedef enum {
      /** Successful call. */
      SUCCESS,
      /** You provided an invalid index. */
      INDEX_ERROR,
      /** The value is out of range (Scanner::Rhomax()). */
      OUT_OF_RANGE,
      /** An error occurred. */
      ACQUISITION_ERROR
    }
    status_t;
    
    
    /**
       The <code>name</code> parameter is useful for distinguishing
       between multiple scanners mounted on your robot. The meaning of
       the other parameters is explained in more detail in the general
       section (Scanner class documentation, scroll your browser
       upwards).
       
       \todo Note that the HAL uses channel numbers instead of names
       for distinguishing between scanners. It would be really cool to
       define the names in the HAL and be able to use them "up" here.
    */
    Scanner(/** proxy object used to retrieve actual data */
	    HAL * hal,
	    /** HAL channel number */
	    int hal_channel,
	    /** name of the scanner */
	    const std::string & name,
	    /** sensor origin wrt robot frame, copied over */
	    const Frame & mount,
	    /** number of scans per measurement */
	    unsigned int nscans,
	    /** maximum range [m] */
	    double rhomax,
	    /** angle of first ray wrt sensor frame [rad] */
	    double phi0,
	    /** angular range swept by measurement [rad] */
	    double phirange);
    
    
    /** \return The scanner's name. */
    const std::string & Name() const { return m_name; }
    
    /**
       \return The current scan as an encapsulated data set unless an
       error occurred during the previous call to Update(), in which
       case a "zero" pointer is returned.
    */
    boost::shared_ptr<Scan> GetScanCopy() const;
    
    /**
       Refresh the scan data. This should be done from within a
       dedicated update-thread to avoid problems when multiple clients
       access the Scanner.
       
       \note This method calls HAL::scan_get() to get the actual
       data. If that doesn't return 0, then GetLocal() and Rho() will
       return ACQUISITION_ERROR, and GetScanCopy() will return a zero
       pointer.
       
       \return The result of the call to HAL::scan_get(), ie 0 on success.
    */
    int Update();
    
    /**
       Get a data point in local coordinates (robot frame).
       \return SUCCESS, OUT_OF_RANGE, INDEX_ERROR, or ACQUISITION_ERROR
    */
    status_t GetLocal(/** index: [0, Scanner::Nscans() - 1] */
		      unsigned int index,
		      /** (return) x-coordinate [m] */
		      double & x,
		      /** (return) y-coordinate [m] */
		      double & y) const;
    
    /**
       Get the distance measurement [m] of a ray in sensor frame.
       \return SUCCESS, OUT_OF_RANGE, INDEX_ERROR, or ACQUISITION_ERROR
    */
    status_t Rho(/** index: [0, Scanner::Nscans() - 1] */
		 unsigned int index,
		 /** (return) distance [m] */
		 double & rho) const;
    
    /**
       Get the angle [rad] of a ray in sensor frame.
       \return SUCCESS or INDEX_ERROR
    */
    status_t Phi(/** index: [0, Scanner::Nscans() - 1] */
		 unsigned int index,
		 /** (return) angle [phi] */
		 double & phi) const;
    
    /**
       Get the cosine of a ray angle in sensor frame.
       \return SUCCESS or INDEX_ERROR
    */
    status_t CosPhi(/** index: [0, Scanner::Nscans() - 1] */
		    unsigned int index,
		    /** (return) angle [phi] */
		    double & cosphi) const;
    
    /**
       Get the sine of a ray angle in sensor frame.
       \return SUCCESS or INDEX_ERROR
    */
    status_t SinPhi(/** index: [0, Scanner::Nscans() - 1] */
		    unsigned int index,
		    /** (return) angle [phi] */
		    double & sinphi) const;
    
    /** \return The number of data points per scan. */
    index_t Nscans() const { return m_nscans; }
    
    /** \return Reference to the sensor's mount point in the robot frame. */
    const Frame & Mount() const { return m_mount; }
    
    /** \return The maximum distance [m]. */
    double Rhomax() const { return m_rhomax; }
    
    /** \return The angle [rad] of the first ray in the sensor frame. */
    double Phi0() const { return m_phi0; }
    
    /** \return The angular range [rad] swept by a scan. */
    double PhiRange() const { return m_phirange; }
    
    /** \note Not meaningfull during ACQUISITION_ERROR! */
    const Timestamp & Tupper() const { return m_scan.m_tupper; }
    
    /** \note Not meaningfull during ACQUISITION_ERROR! */
    const Timestamp & Tlower() const { return m_scan.m_tlower; }
    
    
  protected:
    typedef std::vector<double> vector_t;
    
    HAL * m_hal;
    const int m_hal_channel;
    const std::string m_name;
    const Frame m_mount;
    const size_t m_nscans;
    const double m_rhomax;
    const double m_phi0;
    const double m_phirange;
    const double m_dphi;
    Scan m_scan;
    bool m_data_ok;
    
    vector_t m_cosphi;
    vector_t m_sinphi;
    
    
    void SetRho(size_t i, double r) { m_scan.m_data[i].rho = r; }
    void SetTLower(const Timestamp & t) { m_scan.m_tlower = t; }
    void SetTUpper(const Timestamp & t) { m_scan.m_tupper = t; }
  };
  
}

#endif // SUNFLOWER_SCANNER_HPP
