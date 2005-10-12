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


#include "utOdometry.hpp"
#include "HALstub.hpp"
#include <sstream>
#include <cppunit/Message.h>
using namespace std;
using CppUnit::Message;


CPPUNIT_TEST_SUITE_REGISTRATION( sunflower::utOdometry );


namespace sunflower
{


  void utOdometry::
  setUp()
  {
    _odom = new Odometry();
  }


  void utOdometry::
  tearDown()
  {
    delete _odom;
  }


  void utOdometry::
  testInit()
  {
    Pose pose(187.18729, 12987.91827, 1.98127);

    ostringstream os;
    os << "INFO from utOdometry::testInit():\n>>calling Init()\n";
    try {
      _odom->Init(pose, & os);
    }
    catch(hal_error e){
      os << ">>unexpected hal_error (" << e.retval() << ")\n";
      CPPUNIT_FAIL( os.str() );
    }
    
    Message msg(os.str());
    CPPUNIT_ASSERT_MESSAGE( msg, _odom->Get().X() == pose.X() );
    CPPUNIT_ASSERT_MESSAGE( msg, _odom->Get().Y() == pose.Y() );
    CPPUNIT_ASSERT_MESSAGE( msg, _odom->Get().Theta() == pose.Theta() );

    os << ">>setting HAL error\n";
    HALstub::EmulateError(42);
    os << ">>calling Init()\n";
    try {
      _odom->Init(pose, & os);
      os << ">>should have thrown hal_error (42)\n";
      CPPUNIT_FAIL( os.str() );
    }
    catch(hal_error e){
    }
    HALstub::EmulateError(0);
  }
  

  void utOdometry::
  testUpdate()
  {
    Pose pose1(1.198, 187.1982, -0.123);
    Pose pose2(-18.18276, 10.190821, 2.73);

    ostringstream os;
    os << "INFO from utOdometry::testUpdate():\n>>calling Init()\n";
    CPPUNIT_ASSERT_NO_THROW( _odom->Init(pose1, & os) );
    os << ">>setting HALstub pose\n";
    HALstub::SetPose(pose2);
    os << ">>calling Update()\n";
    try{
      _odom->Update( & os);
    }
    catch(hal_error e){
      os << ">>unexpected hal_error (" << e.retval() << ")\n";
      CPPUNIT_FAIL( os.str() );
    }
    
    Message msg(os.str());
    CPPUNIT_ASSERT_MESSAGE( msg, _odom->Get().X() == pose2.X() );
    CPPUNIT_ASSERT_MESSAGE( msg, _odom->Get().Y() == pose2.Y() );
    CPPUNIT_ASSERT_MESSAGE( msg, _odom->Get().Theta() == pose2.Theta() );
    
    os << ">>setting HAL error\n";
    HALstub::EmulateError(42);
    os << ">>calling Update()\n";
    try{
      _odom->Update( & os);
      os << ">>should have thrown hal_error (42)\n";
      CPPUNIT_FAIL( os.str() );
    }
    catch(hal_error e){
    }
    HALstub::EmulateError(0);
  }


  void utOdometry::
  testGet()
  {
    Pose pose1(1.198, 187.1982, -0.123);
    Pose pose2(-18.18276, 10.190821, 2.73);

    ostringstream os;
    os << "INFO from utOdometry::testGet():\n>>calling Init()\n";
    CPPUNIT_ASSERT_NO_THROW( _odom->Init(pose1, & os) );
    os << ">>calling Set()\n";
    CPPUNIT_ASSERT_NO_THROW( _odom->Set(pose2) );

    Message msg(os.str());
    CPPUNIT_ASSERT_MESSAGE( msg, _odom->Get().X() == pose2.X() );
    CPPUNIT_ASSERT_MESSAGE( msg, _odom->Get().Y() == pose2.Y() );
    CPPUNIT_ASSERT_MESSAGE( msg, _odom->Get().Theta() == pose2.Theta() );
  }


  void utOdometry::
  testGetTime()
  {
    Pose pose1(1.198, 187.1982, -0.123);
    Pose pose2(-18.18276, 10.190821, 2.73);

    ostringstream os;
    os << "INFO from utOdometry::testGetTime():\n>>calling Init()\n";
    CPPUNIT_ASSERT_NO_THROW( _odom->Init(pose1, & os) );
    Timestamp t1(HALstub::GetLastTimestamp());
    os << ">>setting HALstub pose\n";
    HALstub::SetPose(pose2);
    os << ">>calling Update()\n";
    CPPUNIT_ASSERT_NO_THROW( _odom->Update( & os) );
    Timestamp t2(HALstub::GetLastTimestamp());

    try{
      _odom->Get(t1);
    }
    catch(index_error<Timestamp> e){
      ostringstream foo;
      foo << "UNEXPECTED exception in _odom->Get(" << t1 << ")\n"
	  << os.str()
	  << "What: " << e.what() << endl;
      CPPUNIT_FAIL( foo.str() );
    }

    try{
      _odom->Get(t2);
    }
    catch(index_error<Timestamp> e){
      ostringstream foo;
      foo << "UNEXPECTED exception in _odom->Get(" << t2 << ")\n"
	  << os.str()
	  << "What: " << e.what() << endl;
      CPPUNIT_FAIL( foo.str() );
    }

    Message msg(os.str());
    CPPUNIT_ASSERT_MESSAGE( msg, _odom->Get(t1).X() == pose1.X() );
    CPPUNIT_ASSERT_MESSAGE( msg, _odom->Get(t1).Y() == pose1.Y() );
    CPPUNIT_ASSERT_MESSAGE( msg, _odom->Get(t1).Theta() == pose1.Theta() );
    CPPUNIT_ASSERT_MESSAGE( msg, _odom->Get(t2).X() == pose2.X() );
    CPPUNIT_ASSERT_MESSAGE( msg, _odom->Get(t2).Y() == pose2.Y() );
    CPPUNIT_ASSERT_MESSAGE( msg, _odom->Get(t2).Theta() == pose2.Theta() );
  }

  
  void utOdometry::
  testSet()
  {
    Pose pose1(1.198, 187.1982, -0.123);
    Pose pose2(-18.18276, 10.190821, 2.73);

    ostringstream os;
    os << "INFO from utOdometry::testSet():\n>>calling Init()\n";
    CPPUNIT_ASSERT_NO_THROW( _odom->Init(pose1, & os) );
    os << ">>calling Set()\n";
    try{
      _odom->Set(pose2);
    }
    catch(hal_error e){
      os << ">>unexpected hal_error (" << e.retval() << ")\n";
      CPPUNIT_FAIL( os.str() );
    }
    
    Message msg(os.str());
    CPPUNIT_ASSERT_MESSAGE( msg, HALstub::GetPose().X() == pose2.X() );
    CPPUNIT_ASSERT_MESSAGE( msg, HALstub::GetPose().Y() == pose2.Y() );
    CPPUNIT_ASSERT_MESSAGE( msg, HALstub::GetPose().Theta() == pose2.Theta() );

    os << ">>setting HAL error\n";
    HALstub::EmulateError(42);
    os << ">>calling Set()\n";
    try{
      _odom->Set(pose2);
      os << ">>should have thrown hal_error (42)\n";
      CPPUNIT_FAIL( os.str() );
    }
    catch(hal_error e){
    }
    HALstub::EmulateError(0);
  }
  
}
