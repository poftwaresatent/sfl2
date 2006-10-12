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
#include "HAL.hpp"
#include "Scan.hpp"
#include "Timestamp.hpp"
#include <sfl/util/Pthread.hpp>
#include <sfl/util/Frame.hpp>
#include <boost/scoped_array.hpp>
#include <cmath>


using namespace boost;
using namespace std;


namespace sfl {
  
  
  ScannerThread::
  ScannerThread(const string & name)
    : SimpleThread(name)
  {
  }
    
  
  void ScannerThread::
  Step()
  {
    if( ! scanner){
      update_status = -42;
      return;
    }
    update_status = scanner->DoUpdate();
  }
  
  
  Scanner::
  Scanner(shared_ptr<HAL> hal, int _hal_channel, const Frame & _mount,
	  size_t _nscans, double _rhomax, double _phi0, double _phirange,
	  shared_ptr<Mutex> mutex)
    : mount(new Frame(_mount)),
      hal_channel(_hal_channel),
      nscans(_nscans),
      rhomax(_rhomax),
      phi0(_phi0),
      phirange(_phirange),
      dphi(_phirange / _nscans),
      m_hal(hal),
      m_acquisition_ok(false),
      m_cosphi(nscans, 0.0),
      m_sinphi(nscans, 0.0),
      m_mutex(mutex)
  {
    m_buffer.push_back(shared_ptr<Scan>(new Scan(nscans, Timestamp::First(),
						 Timestamp::Last(),
						 Pose())));
    m_buffer.push_back(shared_ptr<Scan>(new Scan(*m_buffer.back())));
    m_dirty = m_buffer[0];
    m_clean = m_buffer[1];
    for(size_t i(0); i < nscans; ++i){
      m_clean->data[i].phi = phi0 + dphi * i;
      m_dirty->data[i].phi = m_clean->data[i].phi;
      m_cosphi[i] = cos(m_clean->data[i].phi);
      m_sinphi[i] = sin(m_clean->data[i].phi);
      m_clean->data[i].rho = rhomax;
      m_dirty->data[i].rho = rhomax;
    }
  }
  
  
  int Scanner::
  Update()
  {
    if(m_thread)
      return m_thread->update_status;
    return DoUpdate();
  }


  bool Scanner::
  SetThread(shared_ptr<ScannerThread> thread)
  {
    Mutex::sentry sentry(m_mutex);
    if(m_thread)
      return false;
    m_thread = thread;
    thread->scanner = this;
    return true;
  }
  
  
  int Scanner::
  DoUpdate()
  {
    // We work with the dirty buffer, and at the end do a quick swap
    // protected by mutex.
    
    scoped_array<double> rho(new double[nscans]);
    struct ::timespec t0, t1;
    size_t actual_nscans(nscans);
    int status(m_hal->scan_get(hal_channel, rho.get(), &actual_nscans,
			       &t0, &t1));
    if(0 != status){
      m_mutex->Lock();
      m_acquisition_ok = false;
      m_mutex->Unlock();
      return status;
    }
    // this probably never happens, but fill in slack with rhomax
    for(size_t ii(actual_nscans); ii < nscans; ++ii)
      rho[ii] = rhomax;
    
    double x, y, theta, sxx, syy, stt, sxy, sxt, syt;
    struct ::timespec foo;
    status = m_hal->odometry_get(&foo, &x, &y, &theta, &sxx, &syy, &stt,
				 &sxy, &sxt, &syt);
    if(0 != status){
      m_mutex->Lock();
      m_acquisition_ok = false;
      m_mutex->Unlock();
      return status;
    }
    
    m_dirty->tlower = t0;
    m_dirty->tupper = t1;
    m_dirty->pose.Set(x, y, theta);
    m_dirty->pose.SetVar(sxx, syy, stt, sxy, sxt, syt);
    for(size_t ii(0); ii < nscans; ++ii){
      m_dirty->data[ii].rho = rho[ii];
      m_dirty->data[ii].locx = rho[ii] * m_cosphi[ii];
      m_dirty->data[ii].locy = rho[ii] * m_sinphi[ii];
      mount->To(m_dirty->data[ii].locx, m_dirty->data[ii].locy);
      m_dirty->data[ii].globx = m_dirty->data[ii].locx;
      m_dirty->data[ii].globy = m_dirty->data[ii].locy;
      m_dirty->pose.To(m_dirty->data[ii].globx, m_dirty->data[ii].globy);
    }
    m_mutex->Lock();
    m_acquisition_ok = true;
    swap(m_dirty, m_clean);
    m_mutex->Unlock();
    
    return 0;
  }
  
  
  Scanner::status_t Scanner::
  GetData(size_t index, scan_data & data) const
  {
    if(index >= nscans)
      return INDEX_ERROR;
    
    Mutex::sentry sentry(m_mutex);
    data = m_clean->data[index];
    if(data.rho >= rhomax)
      return OUT_OF_RANGE;
    return SUCCESS;
  }
  
  
  Scanner::status_t Scanner::
  Phi(size_t index, double & phi) const
  {
    if(index >= nscans)
      return INDEX_ERROR;
    // no need for mutex because phi isn't touched by Update()
    phi = m_clean->data[index].phi;
    return SUCCESS;
  }
  
  
  Scanner::status_t Scanner::
  CosPhi(size_t index, double & cosphi) const
  {
    if(index >= nscans)
      return INDEX_ERROR;
    cosphi = m_cosphi[index];
    return SUCCESS;
  }
  
  
  Scanner::status_t Scanner::
  SinPhi(size_t index, double & sinphi) const
  {
    if(index >= nscans)
      return INDEX_ERROR;
    sinphi = m_sinphi[index];
    return SUCCESS;
  }
  
  
  shared_ptr<Scan> Scanner::
  GetScanCopy() const
  {
    Mutex::sentry sentry(m_mutex);
    return shared_ptr<Scan>(new Scan(*m_clean));
  }
  
  
  const Timestamp & Scanner::
  Tupper() const
  {
    Mutex::sentry sentry(m_mutex);
    return m_clean->tupper;
  }
  
  
  const Timestamp & Scanner::
  Tlower() const
  {
    Mutex::sentry sentry(m_mutex);
    return m_clean->tlower;
  }
  
}
