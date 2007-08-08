/* -*- mode: C++; tab-width: 2 -*- */
/* 
 * Copyright (C) 2007 Roland Philippsen <roland dot philippsen at gmx net>
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


#ifndef SFL_RW_TRAVMAP_HPP
#define SFL_RW_TRAVMAP_HPP


#include <sfl/gplan/TraversabilityMap.hpp>
#include <sfl/util/Pthread.hpp>


namespace sfl {
  
  
  /**
		 Read-only access to a TraversabilityMap, protected by a RWlock.
		 
		 \note Instances of this class are pretty light-weight, they just
		 contain two smart pointers... so don't worry about putting them
		 into smart pointers themselves, just pass around by value or
		 reference.
	*/
  class RDTravmap
  {
  public:
		typedef TraversabilityMap::const_data_t const_data_t;
		
		RDTravmap(boost::shared_ptr<TraversabilityMap const> travmap,
							boost::shared_ptr<RWlock> rwlock)
			: m_travmap(reinterpret_cast<boost::shared_ptr<TraversabilityMap>&>
									(travmap)),
				m_rwlock(rwlock) {}
		
		/** useful for protecting a whole chunk of operations */
		RWlock::rdsentry CreateRDSentry()
		{ return RWlock::rdsentry(m_rwlock); }
				
		/** \note Access through this shared pointer is NOT rwlock protected. */
		const_data_t GetData() const {
			const_data_t data(reinterpret_cast<const_data_t&>(m_travmap->data));
			return data;
		}
		
		GridFrame const & GetGridFrame() const { return m_travmap->gframe;	}
		int GetObstacle() const { return m_travmap->obstacle; }
		int GetFreespace() const { return m_travmap->freespace; }

		/** see TraversabilityMap documentation */
		bool GetValue(double gx, double gy, int & value) const {
			RWlock::rdsentry const sentry(m_rwlock);
			return m_travmap->GetValue(gx, gy, value); }
		
		/** see TraversabilityMap documentation */
		bool GetValue(size_t ix, size_t iy, int & value) const {
			RWlock::rdsentry const sentry(m_rwlock);
			return m_travmap->GetValue(ix, iy, value); }
		
		/** see TraversabilityMap documentation */
		bool GetValue(ssize_t ix, ssize_t iy, int & value) const {
			RWlock::rdsentry const sentry(m_rwlock);
			return m_travmap->GetValue(ix, iy, value); }
		
		/** see TraversabilityMap documentation */
		bool IsObst(double gx, double gy) const {
			RWlock::rdsentry const sentry(m_rwlock);
			return m_travmap->IsObst(gx, gy); }
		
		/** see TraversabilityMap documentation */
		bool IsObst(size_t ix, size_t iy) const {
			RWlock::rdsentry const sentry(m_rwlock);
			return m_travmap->IsObst(ix, iy); }
		
		/** see TraversabilityMap documentation */
		bool IsFree(double gx, double gy) const {
			RWlock::rdsentry const sentry(m_rwlock);
			return m_travmap->IsFree(gx, gy); }
		
		/** see TraversabilityMap documentation */
		bool IsFree(size_t ix, size_t iy) const {
			RWlock::rdsentry const sentry(m_rwlock);
			return m_travmap->IsFree(ix, iy); }
		
		/** see TraversabilityMap documentation */
		void DumpMap(std::ostream * os) const {
			RWlock::rdsentry const sentry(m_rwlock);
			m_travmap->DumpMap(os); }
    
	protected:
		/** \note Not const for subclass WRTravmap! */
		boost::shared_ptr<TraversabilityMap> m_travmap;
		boost::shared_ptr<RWlock> m_rwlock;
  };
	
  
  /**
		 Read-write access to a TraversabilityMap, protected by a RWlock.
		 
		 \note See comments for RDTravmap.
	*/
  class WRTravmap
		: public RDTravmap
  {
  public:
		typedef TraversabilityMap::draw_callback dcb;
		
    WRTravmap(boost::shared_ptr<TraversabilityMap> travmap,
							boost::shared_ptr<RWlock> rwlock)
			: RDTravmap(travmap, rwlock) {}
		
		/** useful for protecting a whole chunk of operations */
		RWlock::wrsentry CreateWRSentry()
		{ return RWlock::wrsentry(m_rwlock); }
		
		/** see TraversabilityMap documentation */
 		bool SetValue(double gx, double gy, int value, dcb * cb) {
			RWlock::wrsentry const sentry(m_rwlock);
			return m_travmap->SetValue(gx, gy, value, cb);
		}
		
		/** see TraversabilityMap documentation */
 		bool SetValue(size_t ix, size_t iy, int value, dcb * cb) {
			RWlock::wrsentry const sentry(m_rwlock);
			return m_travmap->SetValue(ix, iy, value, cb);
		}
		
		/** see TraversabilityMap documentation */
 		bool SetValue(ssize_t ix, ssize_t iy, int value, dcb * cb) {
			RWlock::wrsentry const sentry(m_rwlock);
			return m_travmap->SetValue(ix, iy, value, cb);
		}
		
		/** see TraversabilityMap documentation */
 		bool SetObst(double gx, double gy, dcb * cb) {
			RWlock::wrsentry const sentry(m_rwlock);
			return m_travmap->SetObst(gx, gy, cb);
		}

		/** see TraversabilityMap documentation */
 		bool SetObst(size_t ix, size_t iy, dcb * cb) {
			RWlock::wrsentry const sentry(m_rwlock);
			return m_travmap->SetObst(ix, iy, cb);
		}
		
		/** see TraversabilityMap documentation */
 		bool SetFree(double gx, double gy, dcb * cb) {
			RWlock::wrsentry const sentry(m_rwlock);
			return m_travmap->SetFree(gx, gy, cb);
		}
		
		/** see TraversabilityMap documentation */
 		bool SetFree(size_t ix, size_t iy, dcb * cb) {
			RWlock::wrsentry const sentry(m_rwlock);
			return m_travmap->SetFree(ix, iy, cb);
		}
	};

}

#endif // SFL_RW_TRAVMAP_HPP
