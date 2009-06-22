/* -*- mode: C++; tab-width: 2 -*- */
/* 
 * Copyright (C) 2009 Roland Philippsen <roland dot philippsen at gmx dot net>
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

#ifndef SUNFLOWER_FRAME_FUSION_HPP
#define SUNFLOWER_FRAME_FUSION_HPP

#include <sfl/api/Timestamp.hpp>
#include <sfl/util/ringbuf.hpp>
#include <sfl/util/Frame.hpp>
#include <iostream>

namespace sfl {
  
  
  template<typename data_type>
  struct stamped {
    typedef data_type data_t;
    
    stamped(): tstamp(), data() {}
    stamped(Timestamp const & _tstamp): tstamp(_tstamp), data() {}
    
    stamped(Timestamp const & _tstamp, data_t const & _data)
      : tstamp(_tstamp), data(_data) {}
		
    Timestamp tstamp;
    data_t data;
  };

}

namespace std {
	
	template<typename data_type>
	ostream & operator << (ostream & os, sfl::stamped<data_type> const & rhs) {
		os << "[" << rhs.tstamp << " " << rhs.data << "]";
		return os;
	}

}

namespace sfl {
	
  struct global_speed_s {
    global_speed_s(): sd(0), thetad(0) {}
    global_speed_s(double _sd, double _thetad): sd(_sd), thetad(_thetad) {}
    
    double sd, thetad;
  };
  
  
  class FrameFusion
  {
  public:
    typedef stamped<Frame> frame_t;
    typedef stamped<global_speed_s> speed_t;
    typedef ringbuf<frame_t> framebuf_t;
    typedef ringbuf<speed_t> speedbuf_t;
    
    FrameFusion(size_t buflen_odom,
								size_t buflen_loc,
								size_t buflen_vel_com,
								size_t buflen_vel_act,
								std::ostream * error_os,
								std::ostream * debug_os);
    
    void AddRawOdometry(Timestamp const & tstamp,
												Frame const & pos);
		
    void AddSpeedCommand(Timestamp const & tstamp,
												 double sd, double thetad);
    
    void AddSpeedActual(Timestamp const & tstamp,
												double sd, double thetad);
    
    bool UpdateOdomCorrection(Timestamp const & tstamp,
															Frame const & slampos);
		
		Frame Extrapolate(Timestamp const & tstamp);
    
  protected:
		std::ostream * m_error_os;
		std::ostream * m_debug_os;
		
    framebuf_t m_odom;
    framebuf_t m_loc;
    framebuf_t m_corr;
		
    speedbuf_t m_vel_com;
    speedbuf_t m_vel_act;
	};
  
}

#endif // SUNFLOWER_FRAME_FUSION_HPP