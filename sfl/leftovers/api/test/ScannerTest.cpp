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


#include "ScannerTest.hpp"


CPPUNIT_TEST_SUITE_REGISTRATION( ScannerTest );


void ScannerTest::
setUp()
{
  _scanner = new ScannerStub();
  _scanner->Update();
}


void ScannerTest::
tearDown()
{
  delete _scanner;
}


void ScannerTest::
testUpdate()
{
  _scanner->SimulateAcquisitionError(false);
  CPPUNIT_ASSERT( _scanner->Update() == true);

  _scanner->SimulateAcquisitionError(true);
  CPPUNIT_ASSERT( _scanner->Update() == false);
}


void ScannerTest::
testAccessors()
{
  double d;
  CPPUNIT_ASSERT( _scanner->GetLocal(0, d, d) == ScannerStub::SUCCESS);
  CPPUNIT_ASSERT( _scanner->GetLocal(_scanner->Nscans(), d, d) ==
		  ScannerStub::INDEX_ERROR);
  CPPUNIT_ASSERT( _scanner->Rho(0, d) == ScannerStub::SUCCESS);
  CPPUNIT_ASSERT( _scanner->Rho(_scanner->Nscans(), d) ==
		  ScannerStub::INDEX_ERROR);
  CPPUNIT_ASSERT( _scanner->Phi(0, d) == ScannerStub::SUCCESS);
  CPPUNIT_ASSERT( _scanner->Phi(_scanner->Nscans(), d) ==
		  ScannerStub::INDEX_ERROR);
  CPPUNIT_ASSERT( _scanner->CosPhi(0, d) == ScannerStub::SUCCESS);
  CPPUNIT_ASSERT( _scanner->CosPhi(_scanner->Nscans(), d) ==
		  ScannerStub::INDEX_ERROR);
  CPPUNIT_ASSERT( _scanner->SinPhi(0, d) == ScannerStub::SUCCESS);
  CPPUNIT_ASSERT( _scanner->SinPhi(_scanner->Nscans(), d) ==
		  ScannerStub::INDEX_ERROR);
  
  _scanner->SimulateAcquisitionError(true);
  CPPUNIT_ASSERT( _scanner->Update() == false);
  CPPUNIT_ASSERT( _scanner->GetLocal(0, d, d) ==
		  ScannerStub::ACQUISITION_ERROR);
  CPPUNIT_ASSERT( _scanner->Rho(0, d) == ScannerStub::ACQUISITION_ERROR);
  CPPUNIT_ASSERT( _scanner->Phi(0, d) == ScannerStub::ACQUISITION_ERROR);
  CPPUNIT_ASSERT( _scanner->CosPhi(0, d) == ScannerStub::ACQUISITION_ERROR);
  CPPUNIT_ASSERT( _scanner->SinPhi(0, d) == ScannerStub::ACQUISITION_ERROR);
}
