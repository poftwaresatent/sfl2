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


#include "Lidar.hpp"
#include "ScannerDrawing.hpp"
#include "RobotServer.hpp"
#include "World.hpp"
#include "HAL.hpp"
#include "Random.hpp"
#include <sfl/util/Ray.hpp>
#include <sfl/util/numeric.hpp>
#include <sfl/api/Scanner.hpp>
#include <iostream>


using namespace sfl;
using namespace boost;
using namespace std;


namespace npm {


  Lidar::
  Lidar(const RobotServer * owner, shared_ptr<HAL> hal,
	const Frame & _mount, size_t _nscans, double _rhomax,
	shared_ptr<Scanner> scanner)
    : Sensor(owner),
      nscans(_nscans),
      rhomax(_rhomax),
      mount(new Frame(_mount)),
      m_hal(hal),
      m_scanner(scanner),
      m_global_pose(new Frame(_mount)),
      m_drawing(new ScannerDrawing(this)),
      m_true_rho(nscans, _rhomax),
      m_noisy_rho(nscans, _rhomax)
  {
    if(0 != hal->time_get( & m_t0)){
      cerr << "ERROR in npm::Lidar ctor(): m_hal->time_get() failed.\n";
      exit(EXIT_FAILURE);
    }
    m_t1 = m_t0;
  }
  
  
  void Lidar::
  InitUpdate()
  {
    *m_global_pose = *mount;
    owner->GetTruePose().To(*m_global_pose);
    for(size_t ir(0); ir < nscans; ++ir){
      m_true_rho[ir] = rhomax;
      m_noisy_rho[ir] = rhomax;
    }
    if(0 != m_hal->time_get(&m_t0)){
      cerr << "ERROR in npm::Lidar::InitUpdate(): m_hal->time_get() failed.\n";
      exit(EXIT_FAILURE);
    }
  }
  
  
  void Lidar::
  StepUpdate(const Line & line)
  {
    Ray ray(*m_global_pose);
    for(size_t ir(0); ir < nscans; ++ir){
      double phi;
      if(Scanner::SUCCESS != m_scanner->Phi(ir, phi)){
	cerr << "BUG in npm::Lidar::StepUpdate():"
	     << " m_scanner->Phi() failed (index " << ir << ")\n";
	exit(EXIT_FAILURE);
      }
      ray.SetAngle(m_global_pose->Theta() + phi);

      const double true_rr(ray.Intersect(line));
      if((true_rr > 0) && (true_rr < m_true_rho[ir]))
	m_true_rho[ir] = true_rr;

      const double noisy_rr(true_rr * Random::Uniform(0.95, 1.05));
      if((noisy_rr > 0) && (noisy_rr < m_noisy_rho[ir]))
	m_noisy_rho[ir] = noisy_rr;
    }
  }
  
  
  void Lidar::
  FinalizeUpdate()
  {
    if(0 != m_hal->time_get(&m_t0)){
      cerr << "ERROR in npm::Lidar::FinalizeUpdate():"
	   << " m_hal->time_get() failed.\n";
      exit(EXIT_FAILURE);
    }
  }
  
}
