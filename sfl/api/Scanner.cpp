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


#include "Scanner.hpp"
#include <cmath>


using boost::shared_ptr;
using std::string;
using std::ostream;


namespace sfl {
  
  
  Scanner::
  Scanner(const string & name,
	  const Frame & mount,
	  unsigned int nscans,
	  double rhomax,
	  double phi0,
	  double phirange):
    m_name(name),
    m_mount(mount),
    m_nscans(nscans),
    m_rhomax(rhomax),
    m_phi0(phi0),
    m_phirange(phirange),
    m_dphi(phirange / nscans),
    m_scan(nscans),
    m_data_ok(false)
  {
    for(size_t i(0); i < m_nscans; ++i){
      m_scan.data[i].phi = m_phi0 + m_dphi * i;
      m_cosphi[i] = cos(m_scan.data[i].phi);
      m_sinphi[i] = sin(m_scan.data[i].phi);
      m_scan.data[i].rho = m_rhomax;
    }
  }
  
  
  int Scanner::
  Update(ostream * dbgos)
  {
    const int res(RetrieveData(dbgos));
    if(0 != res){
      m_data_ok = false;
      return res;
    }
    m_data_ok = true;
    
    // cache local coordinates
    for(size_t i(0); i < m_nscans; ++i){
      m_scan.data[i].locx = m_scan.data[i].rho * m_cosphi[i];
      m_scan.data[i].locy = m_scan.data[i].rho * m_sinphi[i];
      m_mount.To(m_scan.data[i].locx, m_scan.data[i].locy);
    }
    
    return 0;
  }
  
  
  Scanner::status_t Scanner::
  GetLocal(unsigned int index,
	   double & x,
	   double & y)
    const
  {
    if( ! m_data_ok)
      return ACQUISITION_ERROR;
    if(index >= m_nscans)
      return INDEX_ERROR;
    
    x = m_scan.data[index].locx;
    y = m_scan.data[index].locy;
    
    if(m_scan.data[index].rho >= m_rhomax)
      return OUT_OF_RANGE;
    return SUCCESS;
  }
  
  
  Scanner::status_t Scanner::
  Rho(unsigned int index,
      double & rho)
    const
  {
    if( ! m_data_ok)
      return ACQUISITION_ERROR;
    if(index >= m_nscans)
      return INDEX_ERROR;
    
    rho = m_scan.data[index].rho;
    
    if(m_scan.data[index].rho >= m_rhomax)
      return OUT_OF_RANGE;    
    return SUCCESS;
  }
  
  
  Scanner::status_t Scanner::
  Phi(unsigned int index,
      double & phi)
    const
  {
    if(index >= m_nscans)
      return INDEX_ERROR;
    phi = m_scan.data[index].phi;
    return SUCCESS;
  }
  
  
  Scanner::status_t Scanner::
  CosPhi(unsigned int index,
	 double & cosphi)
    const
  {
    if(index >= m_nscans)
      return INDEX_ERROR;
    cosphi = m_cosphi[index];
    return SUCCESS;
  }
  
  
  Scanner::status_t Scanner::
  SinPhi(unsigned int index,
	 double & sinphi)
    const
  {
    if(index >= m_nscans)
      return INDEX_ERROR;
    sinphi = m_sinphi[index];
    return SUCCESS;
  }
  
}
