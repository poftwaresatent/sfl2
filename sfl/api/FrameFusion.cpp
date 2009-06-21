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

#include "FrameFusion.hpp"
#include <iostream>
#include <cmath>

namespace sfl {
	
	
	template<typename data_type>
	stamped<data_type> const *
	find_matching_recurse(Timestamp const & tstamp,
												ringbuf<stamped<data_type> > const & buf,
												ssize_t ilow, ssize_t ihigh)
	{
		if ((ilow == ihigh)
				|| (buf[ilow].tstamp <= tstamp))
			return &buf[ilow];
		if (buf[ihigh].tstamp >= tstamp)
			return &buf[ihigh];
		
		ssize_t const isplit((ilow + ihigh) / 2);
		if (buf[isplit].tstamp >= tstamp)
			return find_matching_recurse(tstamp, buf, ilow, isplit);
		return find_matching_recurse(tstamp, buf, isplit, ihigh);
	}
	
	
	/** \todo generalize and extract into header */
	template<typename data_type>
	stamped<data_type> const *
	find_matching(Timestamp const & tstamp,
								ringbuf<stamped<data_type> > const & buf)
	{
		ssize_t const buflen(buf.size());
		if (0 == buflen)
			return 0;
		if (1 == buflen)
			return &buf[0];
		return find_matching_recurse(tstamp, buf, 0, buflen - 1);
	}
	
	
  FrameFusion::
  FrameFusion(size_t buflen_odom,
							size_t buflen_loc,
							size_t buflen_vel_com,
							size_t buflen_vel_act,
							std::ostream * error_os)
    : m_error_os(error_os),
			m_odom(buflen_odom),
      m_loc(buflen_loc),
      m_corr(buflen_loc),
      m_vel_com(buflen_vel_com),
      m_vel_act(buflen_vel_act)
  {
  }
  
  
  void FrameFusion::
  AddRawOdometry(Timestamp const & tstamp,
								 Frame const & pos)
  {
    m_odom.push_back(frame_t(tstamp, pos));
  }
  
  
  void FrameFusion::
  AddSpeedCommand(Timestamp const & tstamp,
									double sd, double thetad)
  {
    m_vel_com.push_back(speed_t(tstamp, global_speed_s(sd, thetad)));
  }
  
  
  void FrameFusion::
  AddSpeedActual(Timestamp const & tstamp,
								 double sd, double thetad)
  {
    m_vel_act.push_back(speed_t(tstamp, global_speed_s(sd, thetad)));
  }
  
  
  bool FrameFusion::
  UpdateOdomCorrection(Timestamp const & tstamp,
											 Frame const & slampos)
  {
    m_loc.push_back(frame_t(tstamp, slampos));
    
    frame_t const * match(find_matching<Frame>(tstamp, m_odom));
		if ( ! match) {
			if (m_error_os)
				*m_error_os << "sfl::FrameFusion::UpdateOdomCorrection(): no odometry for time "
										<< tstamp << "\n";
			return false;
		}
		
    Frame const & raw_odom(match->data);
    double const corr_th(slampos.Theta() - raw_odom.Theta());
    double const cos_corr_th(cos(corr_th));
    double const sin_corr_th(sin(corr_th));
    double const corr_x(slampos.X() - cos_corr_th * raw_odom.X() + sin_corr_th * raw_odom.Y());
    double const corr_y(slampos.Y() - sin_corr_th * raw_odom.X() - cos_corr_th * raw_odom.Y());
    
    m_corr.push_back(frame_t(tstamp, Frame(corr_x, corr_y, corr_th)));
		
		return true;
  }
  
  
  Frame FrameFusion::
	Extrapolate(Timestamp const & tstamp)
	{
		Frame ext;
		if (m_odom.empty()) {
			if (m_error_os)
				*m_error_os << "sfl::FrameFusion::Extrapolate(): no odometry data\n";
			return ext;
		}
		
		frame_t const & latest_odom(m_odom[m_odom.size() - 1]);
		ext.Set(latest_odom.data);
		
		// XXXX TO DO: extrapolate velocities
		return ext;
	}
  
}
