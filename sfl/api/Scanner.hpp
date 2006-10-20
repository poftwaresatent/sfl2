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


#include <sfl/util/Pthread.hpp>
#include <boost/shared_ptr.hpp>
#include <vector>
#include <string>


namespace sfl {
  
  
  class HAL;
  class Scan;
  class scan_data;
  class Timestamp;
  class Frame;
  class Scanner;
  
  
  /**
     Optional update thread for Scanner. If you use one of these,
     Scanner::Update() will only return the status of the previous
     loop of ScannerThread (which will call
     Scanner::DoUpdate()).
  */
  class ScannerThread
    : public SimpleThread
  {
  private:
    ScannerThread(const ScannerThread &);
    
  public:
    /** You still have to call Scanner::SetThread() and
	ScannerThread::Start(). */
    explicit ScannerThread(const std::string & name);
    virtual void Step();
    
  protected:
    friend class Scanner;
    Scanner * scanner;
    int update_status;
  };
  
  
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
  private:
    Scanner(const Scanner &);
    
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
      OUT_OF_RANGE
    }
    status_t;
    
    /**
       The <code>name</code> parameter is useful for distinguishing
       between multiple scanners mounted on your robot. The meaning of
       the other parameters is explained in more detail in the general
       section (Scanner class documentation, scroll your browser
       upwards).
    */
    Scanner(/** proxy object used to retrieve actual data */
	    boost::shared_ptr<HAL> hal,
	    /** HAL channel number */
	    int hal_channel,
	    /** sensor origin wrt robot frame, copied over */
	    const Frame & mount,
	    /** number of scans per measurement */
	    size_t nscans,
	    /** maximum range [m] */
	    double rhomax,
	    /** angle of first ray wrt sensor frame [rad] */
	    double phi0,
	    /** angular range swept by measurement [rad] */
	    double phirange,
	    /** required read/write lock */
	    boost::shared_ptr<Mutex> mutex);
    
    /**
       \return copy of the most recently acquired scan
       
       \note The returned Scan object uses (phi, rho) relative to the
       sensor origin and can still contain readings that are out of
       range. Some consumers of Scan objects expect (phi, rho)
       relative to the robot origin and / or do not handle
       out-of-range readings very well. Consider using
       Multiscanner::CollectScans() or
       Multiscanner::CollectGlobalScans() in such cases.
    */
    boost::shared_ptr<Scan> GetScanCopy() const;
    
    /**
       Refresh the scan data. This can be done from within a dedicated
       ScannerThread, which is convenient for concurrent access
       from several client threads.
       
       This method calls HAL::scan_get() to get the actual data. If
       that doesn't return 0, then the last valid data is returned by
       accessors. You can check the validity of the last Update() by
       checking AcquisitionOk(), which returns true if the previous
       attempt was successful. However, it is better to rely on Scan
       timestamps to check for valid data, because it is conceivable
       (depending on your application) that the acquisition status
       change between your call to AcquisitionOk() and an accessor.
       
       \note Actually, this is just a switch that either calls
       DoUpdate(), or waits until the ScannerThread has done
       that and then returns the most recent status.
       
       \return 0 on success, -42 if there's an issue with running
       ScannerThread (that would be a bug!), or the result of
       the call to HAL::scan_get().
    */
    int Update();
    
    /** Attempt to attach an update thread. Fails if this Scanner
	already has an update thread. */
    bool SetThread(boost::shared_ptr<ScannerThread> thread);
    
    /**
       Get a data point
       \return SUCCESS, OUT_OF_RANGE, or INDEX_ERROR
    */
    status_t GetData(/** index: [0, Scanner::Nscans() - 1] */
		     size_t index, scan_data & data) const;
    
    /**
       Get the angle [rad] of a ray in sensor frame.
       \return SUCCESS or INDEX_ERROR
    */
    status_t Phi(/** index: [0, Scanner::Nscans() - 1] */
		 size_t index,
		 /** (return) angle [phi] */
		 double & phi) const;
    
    /**
       Get the cosine of a ray angle in sensor frame.
       \return SUCCESS or INDEX_ERROR
    */
    status_t CosPhi(/** index: [0, Scanner::Nscans() - 1] */
		    size_t index,
		    /** (return) angle [phi] */
		    double & cosphi) const;
    
    /**
       Get the sine of a ray angle in sensor frame.
       \return SUCCESS or INDEX_ERROR
    */
    status_t SinPhi(/** index: [0, Scanner::Nscans() - 1] */
		    size_t index,
		    /** (return) angle [phi] */
		    double & sinphi) const;
    
    /** \return upper timestamp of the last successfully acquired scan */
    const Timestamp & Tupper() const;
    
    /** \return lower timestamp of the last successfully acquired scan */
    const Timestamp & Tlower() const;
    
    bool AcquisitionOk() const;
    
    const boost::shared_ptr<const Frame> mount;
    const int hal_channel;
    const size_t nscans;
    const double rhomax;
    const double phi0;
    const double phirange;
    const double dphi;
    
    /** true by default, leads to acquisition errors if
	HAL::scan_get() results in fewer points than expected. If set
	to false, then the "slack" will simply be filled with
	rhomax. Beware, a mismatch between expected and actual number
	of scans means that your system is probably not configured
	correctly. */
    bool strict_nscans_check;
    
  protected:
    friend class ScannerThread;
    
    typedef std::vector<double> vector_t;
    
    int DoUpdate();
    
    boost::shared_ptr<HAL> m_hal;
    std::vector<boost::shared_ptr<Scan> > m_buffer;
    boost::shared_ptr<Scan> m_dirty, m_clean; // mutexed
    bool m_acquisition_ok;	// mutexed
    vector_t m_cosphi;
    vector_t m_sinphi;
    boost::shared_ptr<Mutex> m_mutex;
    boost::shared_ptr<ScannerThread> m_thread;
  };
  
}

#endif // SUNFLOWER_SCANNER_HPP
