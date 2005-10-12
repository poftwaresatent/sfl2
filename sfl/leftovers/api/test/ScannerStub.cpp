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


#include "ScannerStub.hpp"


ScannerStub::
ScannerStub():
  sunflower::Scanner("stub",
		     sunflower::Frame(0.5, -3, 0.1),
		     57,	// nscans
		     6.7,	// rhomax
		     0.56,	// phi0
		     4.1),	// phirange
  _simulerror(false)
{
}


void ScannerStub::
SimulateAcquisitionError(bool on)
{
  _simulerror = on;
}


bool ScannerStub::
RetrieveData()
{
  if(_simulerror)
    return false;

  for(sunflower::index_t i(0); i < Nscans(); ++i)
    _rho[i] = i * Rhomax() / (Nscans() - 1);

  return true;
}
