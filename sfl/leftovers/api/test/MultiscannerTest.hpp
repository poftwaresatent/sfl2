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


#ifndef MULTISCANNERTEST_HPP
#define MULTISCANNERTEST_HPP


#include <cppunit/extensions/HelperMacros.h>
#include <sfl/api/Multiscanner.hpp>


class MultiscannerTest:
  public CppUnit::TestFixture
{
  CPPUNIT_TEST_SUITE( MultiscannerTest );
  CPPUNIT_TEST( testMangement );
  CPPUNIT_TEST( testCollect );
  CPPUNIT_TEST_SUITE_END();

public:
  void setUp();
  void tearDown();
  void testMangement();
  void testCollect();
  
private:
  static const unsigned int _nscanners;
  std::vector<sunflower::Scanner *> _scanner;
  sunflower::Multiscanner * _multiscanner;
};

#endif // MULTISCANNERTEST_HPP
