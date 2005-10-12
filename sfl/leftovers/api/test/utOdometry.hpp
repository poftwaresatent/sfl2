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


#ifndef SUNFLOWER_UT_ODOMETRY_HPP
#define SUNFLOWER_UT_ODOMETRY_HPP


#include <cppunit/extensions/HelperMacros.h>
#include <sfl/api/Odometry.hpp>


namespace sunflower
{

  class utOdometry:
    public CppUnit::TestFixture
  {
    CPPUNIT_TEST_SUITE( utOdometry );
    CPPUNIT_TEST( testInit );
    CPPUNIT_TEST( testUpdate );
    CPPUNIT_TEST( testGet );
    CPPUNIT_TEST( testGetTime );
    CPPUNIT_TEST( testSet );
    CPPUNIT_TEST_SUITE_END();
    
  public:
    void setUp();
    void tearDown();
    void testInit();
    void testUpdate();
    void testGet();
    void testGetTime();
    void testSet();

  private:
    Odometry * _odom;
  };
  
}

#endif // SUNFLOWER_UT_ODOMETRY_HPP
