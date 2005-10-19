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
  Scanner(HAL * hal,
	  int hal_channel,
	  const string & name,
	  const Frame & mount,
	  unsigned int nscans,
	  double rhomax,
	  double phi0,
	  double phirange):
    m_hal(hal),
    m_hal_channel(hal_channel),
    m_name(name),
    m_mount(mount),
    m_nscans(nscans),
    m_rhomax(rhomax),
    m_phi0(phi0),
    m_phirange(phirange),
    m_dphi(phirange / nscans),
    m_scan(nscans),
    m_data_ok(false),
    m_cosphi(nscans, 0.0),
    m_sinphi(nscans, 0.0)
  {
    for(size_t i(0); i < m_nscans; ++i){
      m_scan.m_data[i].phi = m_phi0 + m_dphi * i;
      m_cosphi[i] = cos(m_scan.m_data[i].phi);
      m_sinphi[i] = sin(m_scan.m_data[i].phi);
      m_scan.m_data[i].rho = m_rhomax;
    }
  }
  
  
  int Scanner::
  Update()
  {
    // NOTE: It's important to set m_data_ok to true before calling
    // m_hal->scan_get() because in nepumuk's HAL implementation, that
    // ends up querying this Scanner instance about rho!
    m_data_ok = true;
    double rho[m_nscans];
    struct ::timespec t0, t1;
    const int res(m_hal->scan_get(m_hal_channel, rho, m_nscans, &t0, &t1));
    if(0 != res){
      m_data_ok = false;
      return res;
    }
    
    m_scan.m_tlower = t0;
    m_scan.m_tupper = t1;
    for(size_t i(0); i < m_nscans; ++i){
      m_scan.m_data[i].rho = rho[i];
      m_scan.m_data[i].locx = rho[i] * m_cosphi[i];
      m_scan.m_data[i].locy = rho[i] * m_sinphi[i];
      m_mount.To(m_scan.m_data[i].locx, m_scan.m_data[i].locy);
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
    
    x = m_scan.m_data[index].locx;
    y = m_scan.m_data[index].locy;
    
    if(m_scan.m_data[index].rho >= m_rhomax)
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
    
    rho = m_scan.m_data[index].rho;
    
    if(m_scan.m_data[index].rho >= m_rhomax)
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
    phi = m_scan.m_data[index].phi;
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
  

  shared_ptr<Scan> Scanner::
  GetScanCopy()
    const
  {
    return shared_ptr<Scan>(new Scan(m_scan));
  }
  
}
