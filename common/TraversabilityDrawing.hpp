/* -*- mode: C++; tab-width: 2 -*- */
/* 
 * Copyright (C) 2006
 * Swiss Federal Institute of Technology, Zurich. All rights reserved.
 * 
 * Developed at the Autonomous Systems Lab.
 * Visit our homepage at http://www.asl.ethz.ch/
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


#ifndef NPM_TRAVERSABILITY_DRAWING_HPP
#define NPM_TRAVERSABILITY_DRAWING_HPP


#include <npm/common/Drawing.hpp>
#include <sfl/gplan/RWTravmap.hpp>


namespace npm {
  
  class TravProxyAPI {
	public:
		TravProxyAPI(): enable(true) {}
		virtual ~TravProxyAPI() {}
		virtual bool Enabled() const = 0;
		virtual double GetX() const = 0;
		virtual double GetY() const = 0;
		virtual double GetTheta() const = 0;
		virtual double GetDelta() const = 0;
		virtual int GetObstacle() const = 0;
		virtual int GetFreespace() const = 0;
		virtual size_t GetXSize() const = 0;
		virtual size_t GetYSize() const = 0;
		virtual int GetValue(size_t ix, size_t iy) const = 0;
		bool enable;
	};
  
	
  class PtrTravProxy: public TravProxyAPI {
	public:
		PtrTravProxy(boost::shared_ptr<sfl::TraversabilityMap const> travmap);
		virtual bool Enabled() const;
		virtual double GetX() const;
		virtual double GetY() const;
		virtual double GetTheta() const;
		virtual double GetDelta() const;
		virtual int GetObstacle() const;
		virtual int GetFreespace() const;
		virtual size_t GetXSize() const;
		virtual size_t GetYSize() const;
		virtual int GetValue(size_t ix, size_t iy) const;
	protected:
		boost::shared_ptr<sfl::TraversabilityMap const> m_travmap;
	};
  
	
  class RDTravProxy: public TravProxyAPI {
	public:
		RDTravProxy(boost::shared_ptr<sfl::RDTravmap> rdtravmap);
		virtual bool Enabled() const;
		virtual double GetX() const;
		virtual double GetY() const;
		virtual double GetTheta() const;
		virtual double GetDelta() const;
		virtual int GetObstacle() const;
		virtual int GetFreespace() const;
		virtual size_t GetXSize() const;
		virtual size_t GetYSize() const;
		virtual int GetValue(size_t ix, size_t iy) const;
	protected:
		boost::shared_ptr<sfl::RDTravmap> m_rdtravmap;
	};
	
	
  class TraversabilityDrawing
    : public Drawing
  {
  public:
    TraversabilityDrawing(const std::string & name,
													boost::shared_ptr<TravProxyAPI> proxy);
		
		/** \note Packs proxy into a boost::shared_ptr<>, so only use this
				if you have a raw pointer that will NOT be deleted in your
				code. */
    TraversabilityDrawing(const std::string & name,
													TravProxyAPI * proxy);
    
    virtual void Draw();
    
  private:
    boost::shared_ptr<TravProxyAPI> m_proxy;
  };

}

#endif // NPM_TRAVERSABILITY_DRAWING_HPP
