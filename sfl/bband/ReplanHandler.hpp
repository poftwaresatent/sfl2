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
#include <sfl/oa/gplan/NF1.hpp>
#include <sfl/oa/bband/BubbleFactory.hpp>


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

    void UpdateEmulation(boost::shared_ptr<const GlobalScan> scan);
    void StartNewThread();
    void Abort();

    /** \todo Only needed for plotting, should be hidden. */
    int Breakpoint() const { return _breakpoint; }

    /** \todo Only needed for plotting, should be hidden. */
    const BubbleList * BufferBlist() const { return _buffer_blist; }

    /** \todo Only needed for plotting, should be hidden. */
    const BubbleList * InitialBand() const { return _initial_band; }

    /** \todo Only needed for plotting, should be hidden. */
    const NF1 & GetNF1() const { return _nf1; }
    
  private:
    friend class BubbleBand;
    
    static const double DEFAULTNF1WIDTH = 4.0;
    static const int DEFAULTNF1DIMENSION = 21;
    
    BubbleBand & _bubble_band;
    const Odometry & _odometry;
    BubbleFactory & _bubble_factory;
    NF1 & _nf1;
    BubbleList * _buffer_blist;

    /** \todo Only needed for plotting. */
    BubbleList * _initial_band;

    double _nf1width;
    int _nf1dimension;
    int _state;
    char * _statestr[5];

    // for thread emulation
    int _threadcounter;
    int _breakpoint;

    int Run(int & breakpoint, boost::shared_ptr<const GlobalScan> scan);

    bool GeneratePlan(boost::shared_ptr<const GlobalScan> scan);
    bool GenerateBand(boost::shared_ptr<const GlobalScan> scan);
  };

}

#endif // SUNFLOWER_REPLANHANDLER_HPP
