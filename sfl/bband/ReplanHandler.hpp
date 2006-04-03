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


#ifndef SUNFLOWER_REPLANHANDLER_HPP
#define SUNFLOWER_REPLANHANDLER_HPP


#include <sfl/api/Odometry.hpp>
#include <sfl/gplan/NF1.hpp>
#include <sfl/bband/BubbleFactory.hpp>
#include <boost/scoped_ptr.hpp>
#include <string>


namespace sfl {


  class BubbleBand;		// circular dep.
  class BubbleList;		// circular dep.


  class ReplanHandler
  {
  public:
    typedef enum {
      NOTRUNNING  = 0,
      RUNNING,
      EXITSUCCESS,
      EXITFAILURE,
      ABORTED
    } state_t;
    
    
    ReplanHandler(BubbleBand & bubble_band,
		  const Odometry & odometry,
		  BubbleFactory & bubble_factory);
    ~ReplanHandler();
    
    /** Update the internal state based on pending replanning requests
	or ongoing plannings.
	
	\note The Scan object should be filtered, ie contain only
	valid readings. This can be obtained from
	Multiscanner::CollectScans() and
	Multiscanner::CollectGlobalScans(), whereas
	Scanner::GetScanCopy() can still contain readings that are out
	of range (represented as readings at the maximum rho
	value). */
    void Update(boost::shared_ptr<const GlobalScan> scan);
    
    /** Sets up the internal state such that the next call to Update()
	will result in a new plan, if available. */
    void StartPlanning();
    
    void Abort();
    
    /** Returns the "buffer" bubble list that contains the initial
	bubble band if called at the right moment (ie if GetState() ==
	EXITSUCCESS). The list passed as parameters is emptied via
	BubbleList::RemoveAll() and henceforth used as internal
	buffer. This mechanism is designed for BubbleBand
	implementation. */
    BubbleList * SwapBubbleList(BubbleList * replace);
    
    /** \return The current state <b>and possibly change it</b>,
	acting as a latch (this is designed to work with BubbleBand
	implementation):
	<ul><li> EXITSUCCESS ==> NOTRUNNING </li>
	    <li> EXITFAILURE ==> NOTRUNNING </li></ul>
    */
    state_t GetState();
    static const std::string & GetStateName(state_t state);
    
    /** \todo Only needed for plotting, should be hidden. */
    const BubbleList * BufferBlist() const { return m_buffer_blist; }
    
    /** \todo Only needed for plotting, should be hidden. */
    const BubbleList * InitialBand() const { return m_initial_band.get(); }
    
    /** \todo Only needed for plotting, should be hidden. */
    const NF1 & GetNF1() const { return * m_nf1; }
    
    
  private:
    static const double DEFAULTNF1WIDTH = 4.0;
    static const int DEFAULTNF1DIMENSION = 21;
    
    BubbleBand & m_bubble_band;
    const Odometry & m_odometry;
    BubbleFactory & m_bubble_factory;
    boost::scoped_ptr<NF1> m_nf1;
    BubbleList * m_buffer_blist;
    
    /** \note only needed for debug plotting. */
    boost::scoped_ptr<BubbleList> m_initial_band;
    
    double m_nf1width;
    int m_nf1dimension;
    state_t m_state;
    
    
    bool GeneratePlan(boost::shared_ptr<const GlobalScan> scan);
    bool GenerateBand(boost::shared_ptr<const GlobalScan> scan);
  };

}

#endif // SUNFLOWER_REPLANHANDLER_HPP
