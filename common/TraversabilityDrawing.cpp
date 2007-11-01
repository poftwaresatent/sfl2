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


#include "TraversabilityDrawing.hpp"
#include "wrap_gl.hpp"
#include "Manager.hpp"
#include <sfl/util/numeric.hpp>
#include <sfl/util/pdebug.hpp>
#include <sfl/gplan/TraversabilityMap.hpp>
#include <math.h>


#ifdef ASL_DEBUG
# define PDEBUG PDEBUG_OUT
#else // ! ASL_DEBUG
# define PDEBUG PDEBUG_OFF
#endif // ASL_DEBUG
#define PVDEBUG PDEBUG_OFF


using namespace sfl;
using namespace boost;
using namespace std;


namespace npm {
  
  
  TraversabilityDrawing::
  TraversabilityDrawing(const string & name,
												shared_ptr<TravProxyAPI> proxy)
    : Drawing(name,
							"traversability map as greyscale with special highlights",
							Instance<UniqueManager<Drawing> >()),
			m_proxy(proxy)
  {
  }
  
  
  TraversabilityDrawing::
  TraversabilityDrawing(const string & name,
												TravProxyAPI * proxy)
    : Drawing(name,
							"traversability map as greyscale with special highlights",
							Instance<UniqueManager<Drawing> >()),
			m_proxy(proxy)
  {
  }
  
  
  void TraversabilityDrawing::
  Draw()
  {
		if( ! m_proxy->Enabled())
			return;
    
    glMatrixMode(GL_MODELVIEW);
    glPushMatrix();
    glTranslated(m_proxy->GetX(), m_proxy->GetY(), 0);
    glRotated(180 * m_proxy->GetTheta() / M_PI, 0, 0, 1);
    glScaled(m_proxy->GetDelta(), m_proxy->GetDelta(), 1);
    
    glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
    const double
			cscale(1.0 / (m_proxy->GetObstacle() - m_proxy->GetFreespace()));
    for(size_t ix(0); ix < m_proxy->GetXSize(); ++ix)
      for(size_t iy(0); iy < m_proxy->GetYSize(); ++iy){
				const int value(m_proxy->GetValue(ix, iy));
				if(value > m_proxy->GetObstacle())
					glColor3d(0.5, 0, 0.6);
				else if(value == m_proxy->GetObstacle())
					glColor3d(0.5, 0, 0);
				else if(value < m_proxy->GetFreespace())
					glColor3d(0, 0, 0.5);
				else if(value == m_proxy->GetFreespace())
					glColor3d(0, 0.5, 0);
				else{
					const double grey((value - m_proxy->GetFreespace()) * cscale);
					glColor3d(grey, grey, grey);
				}
				glRectd(ix - 0.5, iy - 0.5, ix + 0.5, iy + 0.5);
      }
    glMatrixMode(GL_MODELVIEW);
    glPopMatrix();
	}
	
	
	PtrTravProxy::
	PtrTravProxy(shared_ptr<TraversabilityMap const> travmap)
		: m_travmap(travmap)
	{
	}
	
	
	bool PtrTravProxy::
	Enabled() const
	{
		if( ! enable)
			return false;
    if( ! m_travmap)
      return false;
    if( ! m_travmap->data)
      return false;
		return true;
	}
	
	
	double PtrTravProxy::
	GetX() const
	{
		return m_travmap->gframe.X();
	}
	
	
	double PtrTravProxy::
	GetY() const
	{
		return m_travmap->gframe.Y();
	}
	
	
	double PtrTravProxy::
	GetTheta() const
	{
		return m_travmap->gframe.Theta();
	}
	
	
	double PtrTravProxy::
	GetDelta() const
	{
		return m_travmap->gframe.Delta();
	}
	
	
	int PtrTravProxy::
	GetObstacle() const
	{
		return m_travmap->obstacle;
	}
	
	
	int PtrTravProxy::
	GetFreespace() const
	{
		return m_travmap->freespace;
	}
	
	
	size_t PtrTravProxy::
	GetXSize() const
	{
		return m_travmap->data->xsize;
	}
	
	
	size_t PtrTravProxy::
	GetYSize() const
	{
		return m_travmap->data->ysize;
	}
	
	
	int PtrTravProxy::
	GetValue(size_t ix, size_t iy) const
	{
		return (*m_travmap->data)[ix][iy];
	}
  
	
	RDTravProxy::
	RDTravProxy(boost::shared_ptr<sfl::RDTravmap> rdtravmap)
		: m_rdtravmap(rdtravmap)
	{
	}
	
	
	bool RDTravProxy::
	Enabled() const
	{
		if( ! enable)
			return false;
    if( ! m_rdtravmap->GetData())
      return false;
		return true;
	}
	
	
	double RDTravProxy::
	GetX() const
	{
		return m_rdtravmap->GetGridFrame().X();
	}
	
	
	double RDTravProxy::
	GetY() const
	{
		return m_rdtravmap->GetGridFrame().Y();
	}
	
	
	double RDTravProxy::
	GetTheta() const
	{
		return m_rdtravmap->GetGridFrame().Theta();
	}
	
	
	double RDTravProxy::
	GetDelta() const
	{
		return m_rdtravmap->GetGridFrame().Delta();
	}
	
	
	int RDTravProxy::
	GetObstacle() const
	{
		return m_rdtravmap->GetObstacle();
	}
	
	
	int RDTravProxy::
	GetFreespace() const
	{
		return m_rdtravmap->GetFreespace();
	}
	
	
	size_t RDTravProxy::
	GetXSize() const
	{
		return m_rdtravmap->GetData()->xsize;
	}
	
	
	size_t RDTravProxy::
	GetYSize() const
	{
		return m_rdtravmap->GetData()->ysize;
	}
	
	
	int RDTravProxy::
	GetValue(size_t ix, size_t iy) const
	{
		int result(0);
		m_rdtravmap->GetValue(ix, iy, result);
		return result;
	}

}
