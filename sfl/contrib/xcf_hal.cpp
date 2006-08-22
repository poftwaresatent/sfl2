/*
 * Copyright (c) 2005 Universitaet Bielefeld, Germany
 *                    and LAAS/CNRS, France
 *
 * Authors: Thorsten Spexard <tspexard at techfak dot uni-bielefeld dot de>
 *          Roland Philippsen <roland dot philippsen at gmx dot net>
 *
 * Permission to use, copy, modify, and distribute this software for any
 * purpose with or without fee is hereby granted, provided that the above
 * copyright notice and this permission notice appear in all copies.
 *
 * THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
 * WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
 * ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
 * WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
 * ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
 * OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
 */


#include "xcf_hal.h"
#include "laserdata.h"
#include "odometrydata.h"
#include "movementdata.h"
#include "AttTarget.h"
#include "esvdata.h"

#include <sfl/util/numeric.hpp>
#include <sys/time.h>
#include <xcf/xcf.hpp>
#include <xmltio/xmltio.hpp>

#include <sfl/util/pdebug.hpp>
#define PDEBUG PDEBUG_OUT

#include <boost/shared_ptr.hpp>
#include <sstream>


using namespace sfl;
using namespace XCF;
using namespace xmltio;
using namespace boost;
using namespace std;


/** When != 0, don't do anything but just write messages about what
    would be done. Set through xcf_hal_dryrun_on(), reset with
    xcf_hal_dryrun_off(). */
static FILE * debugstream(0);

/** Used for recording error messages from XCF exceptions. */
static shared_ptr<ostringstream> error_os;

static SubscriberPtr odometry_subscriber;
static PublisherPtr  speed_publisher;
static SubscriberPtr laser_subscriber;
static SubscriberPtr navgoal_subscriber;
static PublisherPtr  navresult_publisher;

static string schema_dir(string(getenv("BIRON")) + "/cfg/schemas/");
static string speed_schema_name("MoveMsgKE1.xsd");
static string navresult_schema_name("EventMsgKE1.xsd");
static string laser_publisher_name("LaserData");
static string odometry_publisher_name("OdometryData");
static string speed_publisher_name("NAV_RelPos_Publisher");
static string navgoal_publisher_name("SS-NAV");
static string navresult_publisher_name("EQ-NAV");


int xcf_odometry_init()
{
  if(0 != debugstream){
    fprintf(debugstream, "DEBUG xcf_odometry_init()\n");
    return 0;
  }
  try{
    odometry_subscriber = Subscriber::create(odometry_publisher_name);
    odometry_subscriber->setOnlyReceiveLast(true);
    PDEBUG("subscribed to %s\n", odometry_publisher_name.c_str());
  }
  catch(const GenericException & ex){
    error_os.reset(new ostringstream());
    (*error_os) << "xcf_odometry_init(): " << ex.reason;
    return -1;
  }
  return 0;
}


int xcf_odometry_receive(double * x, double * y, double * theta,
			 uint64_t * timestamp_ms)
{
  if(0 != debugstream){
    fprintf(debugstream, "DEBUG xcf_odometry_receive()\n");
    *x = 0;
    *y = 0;
    *theta = 0;
    *timestamp_ms = 0;
    return 0;
  }
  string message;
  try{
    message = odometry_subscriber->receive(true);
  }
  catch(PublisherEmptyException){
    return 1;
  }
  tOdometryData data(extract<tOdometryData>(Location(message, "/MSG")));
  * x = data.fXCoord;
  * y = data.fYCoord;
  * theta = data.fYaw;
  * timestamp_ms = data.lTimeStamp;
  return 0;
}


void xcf_odometry_end()
{
  if(0 != debugstream)
    fprintf(debugstream, "DEBUG xcf_odometry_end()\n");
  else
    odometry_subscriber->destroy();
}


int xcf_speed_init()
{
  if(0 != debugstream){
    fprintf(debugstream, "DEBUG xcf_speed_init()\n");
    return 0;
  }
  try{
    speed_publisher = Publisher::create(speed_publisher_name);
    speed_publisher->setSchema(schema_dir + speed_schema_name);
    PDEBUG("publishing on %s\n", speed_publisher_name.c_str());
  }
  catch(const GenericException & ex){
    error_os.reset(new ostringstream());
    (*error_os) << "xcf_speed_init(): " << ex.reason;
    return -1;
  }
  return 0;
}


int xcf_speed_send(double v, double w)
{
  if(0 != debugstream){
    fprintf(debugstream, "DEBUG xcf_speed_send(%f, %f)\n", v, w);
    return 0;
  }
  t_MovementData data;
  data.lTimeStamp = xcf_hal_timestamp();
  if(absval(v) < 1e-3)
    data.dVTrans = 0;
  else
    data.dVTrans = v;
  if(absval(w) < 1e-3)
    data.dVRot = 0;
  else
    data.dVRot = w;
  data.sGenerator = "NAV";  
  Location dataloc(sMovementData_tmpl, "/MSG");
  dataloc = data;
  try{
    speed_publisher->send(dataloc.getDocumentText());
  }
  catch(const ValidateFailedException & vex){
    error_os.reset(new ostringstream());
    (*error_os) << "xcf_speed_send(): " << vex.reason << "\n"
		<< dataloc.getDocumentText();
    return -1;
  }
  return 0;
}


void xcf_speed_end()
{
  if(0 != debugstream)
    fprintf(debugstream, "DEBUG xcf_speed_end()\n");
  else
    speed_publisher->destroy();
}


int xcf_scan_init()
{
  if(0 != debugstream){
    fprintf(debugstream, "DEBUG xcf_scan_init()\n");
    return 0;
  }
  try{
    laser_subscriber = Subscriber::create(laser_publisher_name);
    laser_subscriber->setOnlyReceiveLast(true);
    PDEBUG("subscribed to %s\n", laser_publisher_name.c_str());
  }
  catch(const GenericException & ex){
    error_os.reset(new ostringstream());
    (*error_os) << "xcf_scan_init(): " << ex.reason;
    return -1;
  }
  return 0;
}


int xcf_scan_receive(double * rho, int * rho_len, uint64_t * timestamp_ms)
{
  if(0 != debugstream){
    fprintf(debugstream, "DEBUG xcf_scan_receive(.., %d, ..)\n", * rho_len);
    for(int ii(0); ii < * rho_len; ++ii)
      rho[ii] = 0.9;
    return 0;
  }
  string message;
  try{
    message = laser_subscriber->receive(true);
  }
  catch(PublisherEmptyException){
    return 1;
  }
  tLaserData data(extract<tLaserData>(Location(message, "/MSG")));
  if(data.iLaserPoints < * rho_len)
    * rho_len = data.iLaserPoints;
  for(int ii(0); ii < * rho_len; ++ii)
    rho[ii] = data.fPoints[ii];
  * timestamp_ms = data.lTimeStamp;
  return 0;
}


void xcf_scan_end()
{
  if(0 != debugstream)
    fprintf(debugstream, "DEBUG xcf_scan_end()\n");
  else
    laser_subscriber->destroy();
}


int xcf_navgoal_init()
{
  if(0 != debugstream){
    fprintf(debugstream, "DEBUG xcf_navgoal_init()\n");
    return 0;
  }
  try{
    navgoal_subscriber = Subscriber::create(navgoal_publisher_name);
    navgoal_subscriber->setOnlyReceiveLast(false);
    PDEBUG("subscribed to %s\n", navgoal_publisher_name.c_str());
  }
  catch(const GenericException & ex){
    error_os.reset(new ostringstream());
    (*error_os) << "xcf_navgoal_init(): " << ex.reason;
    return -1;
  }
  return 0;
}


int xcf_navgoal_receive(char * location, size_t location_len,
			int * transaction_id, char * esv_state,
			size_t esv_state_len)
{
  if(0 != debugstream){
    fprintf(debugstream, "DEBUG xcf_navgoal_receive()\n");
    strncpy(location, "Kitchen", location_len);
    *transaction_id = 1;
    strncpy(esv_state, "GoTo", esv_state_len);
    return 0;
  }
  string message;
  try{
    message = navgoal_subscriber->receive(true);
  }
  catch(PublisherEmptyException){
    return 1;
  }
  tStatusData data(extract<tStatusData>(Location(message, "/MSG")));
  if((data.sName !="GotoLabel") && (data.sName != "StopNav"))
    return 1;
  strncpy(location, data.sData.c_str(), location_len);
  *transaction_id = data.iOriginNum;
  strncpy(esv_state, data.sState.c_str(), esv_state_len);
  return 0;    
}


void xcf_navgoal_end()
{
  if(0 != debugstream)
    fprintf(debugstream, "DEBUG xcf_navgoal_end()\n");
  else
    navgoal_subscriber->destroy();
}


int xcf_navresult_init()
{
  if(0 != debugstream){
    fprintf(debugstream, "DEBUG xcf_navresult_init()\n");
    return 0;
  }
  try{
    navresult_publisher = Publisher::create(navresult_publisher_name);
    navresult_publisher->setSchema(schema_dir + navresult_schema_name);
    PDEBUG("publishing on %s\n", navresult_publisher_name.c_str());    
  }
  catch(const GenericException & ex){
    error_os.reset(new ostringstream());
    (*error_os) << "xcf_navresult_init(): " << ex.reason;
    return -1;
  }
  return 0;
}


int xcf_navresult_send(const char * result, int transaction_id,
		       const char * esv_state)
{
  if(0 != debugstream){
    fprintf(debugstream, "DEBUG xcf_navresult_send(%s, %d, %s)\n",
	    result, transaction_id, esv_state);
    return 0;
  }
  tEventData data;
  data.lTimeStamp = xcf_hal_timestamp();
  data.sGenerator = "NAV"; 
  data.sOriginMod = "NAV";
  static int inum(0);
  data.iOriginNum = ++inum;
  data.sRefMod = "DLG";
  data.iRefNum = transaction_id;
  data.sName = string(result);
  data.sState = string(esv_state);
  data.lBestBefore = data.lTimeStamp + 500;
  Location dataloc(sEventData_tmpl,"/MSG");
  dataloc = data;
  try{
    navresult_publisher->send(dataloc.getDocumentText());
  }
  catch(const ValidateFailedException & vex){
    error_os.reset(new ostringstream());
    (*error_os) << "xcf_navresult_send(): " << vex.reason << "\n"
		<< dataloc.getDocumentText();
    return -1;
  }
  return 0;
}


void xcf_navresult_end()
{
  if(0 != debugstream)
    fprintf(debugstream, "DEBUG xcf_navresult_end()\n");
  else
    navresult_publisher->destroy();
}


void xcf_hal_dryrun_on(FILE * stream)
{
  debugstream = stream;
}


void xcf_hal_dryrun_off()
{
  debugstream = 0;
}


const char * xcf_hal_geterror()
{
  if( ! error_os)
    return NULL;
  return error_os->str().c_str();
}


uint64_t xcf_hal_timestamp()
{
  timeval t0;
  gettimeofday( & t0, NULL);
  uint64_t tms;
  tms  = static_cast<uint64_t>(t0.tv_sec  * 1000);
  tms += static_cast<uint64_t>(t0.tv_usec / 1000);
  return tms;
}
