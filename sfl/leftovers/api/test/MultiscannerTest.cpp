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


#include "MultiscannerTest.hpp"
#include "ScannerStub.hpp"
#include <sfl/api/numeric.hpp>
using namespace sunflower;


CPPUNIT_TEST_SUITE_REGISTRATION( MultiscannerTest );


const unsigned int MultiscannerTest::_nscanners(3);


void MultiscannerTest::
setUp()
{
  _multiscanner = new Multiscanner("MultiscannerTest");
  
  _scanner.reserve(_nscanners);
  for(unsigned int i(0); i < _nscanners; ++i){
    ScannerStub * scanner = new ScannerStub();
    scanner->Update();
    _scanner.push_back(scanner);
    _multiscanner->Add(scanner);
  }
}


void MultiscannerTest::
tearDown()
{
  for(unsigned int i(0); i < _nscanners; ++i)
    delete _scanner[i];
  delete _multiscanner;
}


void MultiscannerTest::
testMangement()
{
  CPPUNIT_ASSERT( _multiscanner->Nscanners() == _nscanners );
  for(index_t i(0); i < _nscanners; ++i){
    CPPUNIT_ASSERT( _multiscanner->GetScanner(i) == _scanner[i] );
  }
}


void MultiscannerTest::
testCollect()
{
  Scan scan(_multiscanner->CollectScans());
  CPPUNIT_ASSERT( scan.Nscans() <= _nscanners * _scanner[0]->Nscans() );
  
  const Scan::vector_t & phi(scan.Phi());
  const Scan::vector_t & rho(scan.Rho());
  const Scan::vector_t & locx(scan.LocalX());
  const Scan::vector_t & locy(scan.LocalY());
  for(index_t i(0); i < scan.Nscans(); ++i){
    CPPUNIT_ASSERT( absval( rho[i] * cos(phi[i]) - locx[i]) < epsilon );
    CPPUNIT_ASSERT( absval( rho[i] * sin(phi[i]) - locy[i]) < epsilon );
  }
}
