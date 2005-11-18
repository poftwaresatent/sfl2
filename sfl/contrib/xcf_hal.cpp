/*
 * Copyright (c) 2005 Universitaet Bielefeld, Germany
 *                    and LAAS/CNRS, France
 *
 * Authors: Thorsten Spexard <tspexard@techfak.uni-bielefeld.de>
 *          Roland Philippsen <roland.philippsen@gmx.net>
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

#include <sfl/util/numeric.hpp>
#include <sys/time.h>
#include <iostream>
#include <xcf/xcf.hpp>
#include <xmltio/xmltio.hpp>

#include <laserdata.h>
#include <odometrydata.h>
#include <movementdata.h>
#include <esvdata.h>

/**
 * schema
 */
#define MOVESCHEMA            "MoveMsgKE1.xsd"
#define EQSCHEMA              "EventMsgKE1.xsd"

/**
 * default parameter for xcf communication
 */
#define DEFAULT_LASER_PUBLISHER_NAME "LaserData"

/**
 * default parameter for xcf communication
 */
#define DEFAULT_ODOMETRY_PUBLISHER_NAME "OdometryData"

/**
 * default parameter for xcf communication
 */
#define DEFAULT_REL_POS_PUBLISHER_NAME "NAV_RelPos_Publisher"

/**
 * default parameter for xcf communication
 */
#define DEFAULT_ESV2NAV_PUBLISHER_NAME "SS-NAV"

/**
 * default parameter for xcf communication
 */
#define DEFAULT_NAV2ESV_PUBLISHER_NAME "EQ-NAV"


using namespace std;
using namespace XCF;
using namespace xmltio;


/** When != 0, don't do anything but just write messages about what
    would be done. Set through xcf_hal_dryrun_on(), reset with
    xcf_hal_dryrun_off(). */
static FILE * debugstream(0);

static XCF::SubscriberPtr odo_subscriber;
static tOdometryData odo;
static XCF::PublisherPtr _RelPosPublisher;
static XCF::SubscriberPtr laser_subscriber;
static tLaserData ld;
static XCF::SubscriberPtr status_subscriber;
static tStatusData sd;
static XCF::PublisherPtr _EQPublisher;

static string SCHEMAPATH = string(getenv("BIRON")) + "/cfg/schemas/";


int xcf_odometry_init()
{
  if(0 != debugstream){
    fprintf(debugstream, __FILE__ ": DEBUG xcf_odometry_init()\n");
    return 0;
  }
  
  // register xcf-subscriber and connect to xcf-publisher
  try {
    odo_subscriber =
      XCF::Subscriber::create(DEFAULT_ODOMETRY_PUBLISHER_NAME);
    // no message cueing
    odo_subscriber->setOnlyReceiveLast(true);
    cout << "Connected to StreamServer" << endl;
  }
  catch(const XCF::GenericException& ex) {
    cerr << "Error at registering odometry-xcf-subscriber"
	 << endl << "XCF::GenericException: " << ex.reason << endl;
    return -1;
  }
  return 0;
}


int xcf_odometry_get(double * x, double * y, double * theta,
		     uint64_t * timestamp_ms)
{
  if(0 != debugstream){
    fprintf(debugstream, __FILE__ ": DEBUG xcf_odometry_get()\n");
    *x = 0;
    *y = 0;
    *theta = 0;
    *timestamp_ms = 0;
    return 0;
  }
  
  string msg_odo;
  try {
    // get message asynchronous from server
    msg_odo = odo_subscriber->receive(true);
  }
  catch (XCF::PublisherEmptyException) {
    //cerr << "publisher empty" << endl;
    *x = odo.fXCoord;
    *y = odo.fYCoord;
    *theta = odo.fYaw;
    *timestamp_ms = odo.lTimeStamp;
    return 0;
  }
  odo = extract<tOdometryData>(Location(msg_odo,"/MSG"));
  *x = odo.fXCoord;
  *y = odo.fYCoord;
  *theta = odo.fYaw;
  *timestamp_ms = odo.lTimeStamp;
  return 0;
}


int xcf_odometry_end()
{
  if(0 != debugstream){
    fprintf(debugstream, __FILE__ ": DEBUG xcf_odometry_end()\n");
    return 0;
  }
  
  odo_subscriber->destroy();
  return 0;
}


int xcf_speed_init()
{
  if(0 != debugstream){
    fprintf(debugstream, __FILE__ ": DEBUG xcf_speed_init()\n");
    return 0;
  }
  
  // register new xcf-publishers
  try {
    _RelPosPublisher
      = XCF::Publisher::create(DEFAULT_REL_POS_PUBLISHER_NAME);
    _RelPosPublisher->setSchema(string(SCHEMAPATH)+MOVESCHEMA);
  }
  catch(const XCF::GenericException& ex) {
    cerr << "Error at registering xcf-publisher"
	 << endl << "XCF::GenericException: " << ex.reason << endl;
    return -1;
  }
  return 0;
}


int xcf_speed_set(double v, double w)
{
  if(0 != debugstream){
    fprintf(debugstream, __FILE__ ": DEBUG xcf_speed_set(%f, %f)\n", v, w);
    return 0;
  }
  
  t_MovementData mvd;
  
  // calculate recent time
  timeval tCurTime;
  gettimeofday(&tCurTime, NULL);
  
  // converting time to ms since 1.1.1970
  uint64_t lMSecs;
  lMSecs  = static_cast<uint64_t>(tCurTime.tv_sec  * 1000);
  lMSecs += static_cast<uint64_t>(tCurTime.tv_usec / 1000);
  mvd.lTimeStamp = lMSecs;
  if(sfl::absval(v) < 1e-3)
    mvd.dVTrans = 0;
  else
    mvd.dVTrans = v;
  if(sfl::absval(w) < 1e-3)
    mvd.dVRot = 0;
  else
    mvd.dVRot = w;

  fprintf(debugstream, __FILE__ ": WARNING xcf_speed_set(): using zero\n");
  mvd.dVTrans = 0;
  mvd.dVRot = 0;
  
  mvd.sGenerator = "NAV";  

  //converting struct to xml
  Location mvdl(sMovementData_tmpl,"/MSG");
  mvdl = mvd;
  
  //sending xml string
  try
    {
      _RelPosPublisher->send(mvdl.getDocumentText());
    }
  catch(const XCF::ValidateFailedException& vex) 
    {
      cerr << "Error at XML Validataion"
	   << endl << "XCF::ValidateFailedException " << vex.reason << endl;
      cerr << mvdl.getDocumentText() << endl;
      return -1;
    }
  return 0;
}


int xcf_speed_end()
{
  if(0 != debugstream){
    fprintf(debugstream, __FILE__ ": DEBUG xcf_speed_end()\n");
    return 0;
  }
  
  _RelPosPublisher->destroy();
  return 0;
}


int xcf_scan_init(int channel)
{
  if(0 != debugstream){
    fprintf(debugstream, __FILE__ ": DEBUG xcf_scan_init(%d)\n", channel);
    return 0;
  }
  
  // register xcf-subscriber and connect to xcf-publisher
  try {
    laser_subscriber = XCF::Subscriber::create(DEFAULT_LASER_PUBLISHER_NAME);
    // no message cueing
    laser_subscriber->setOnlyReceiveLast(true);
    cout << "Connected to StreamServer" << endl;
  }
  catch(const XCF::GenericException& ex) {
    cerr << "Error at registering laserdata-xcf-subscriber"
	 << endl << "XCF::GenericException: " << ex.reason << endl;
    return -1;
  }
  return 0;
}


int xcf_scan_get(int channel, double * rho, int rho_len,
		 uint64_t * timestamp_ms)
{
  if(0 != debugstream){
    fprintf(debugstream, __FILE__ ": DEBUG xcf_scan_get(%d, .., %d, ..)\n",
	    channel, rho_len);
    for(int i(0); i < rho_len; ++i)
      rho[i] = 0.9;
    return 0;
  }
  
  string msg_laser;
  int i;
  try {
    // get message asynchronous from server
    msg_laser = laser_subscriber->receive(true);
  }
  catch (XCF::PublisherEmptyException) {
    //cerr << "publisher empty" << endl;
    for (i = 0; i < rho_len; i++)
      {
	rho[i] = ld.fPoints[i];
      }
    *timestamp_ms = ld.lTimeStamp;
    return 0;
  }
  ld = extract<tLaserData>(Location(msg_laser,"/MSG"));
  for (i = 0; i < rho_len; i++)
    {
      rho[i] = ld.fPoints[i];
    }
  *timestamp_ms = ld.lTimeStamp;
  return 0;
}


int xcf_scan_end(int channel)
{
  if(0 != debugstream){
    fprintf(debugstream, __FILE__ ": DEBUG xcf_scan_end(%d)\n", channel);
    return 0;
  }
  
  laser_subscriber->destroy();
  return 0;
}

int xcf_navgoal_init()
{
  if(0 != debugstream){
    fprintf(debugstream, __FILE__ ": DEBUG xcf_navgoal_init()\n");
    return 0;
  }
  
  // register xcf-subscriber and connect to xcf-publisher
  try {
    status_subscriber = XCF::Subscriber::create(DEFAULT_ESV2NAV_PUBLISHER_NAME);
    // no message cueing
    status_subscriber->setOnlyReceiveLast(true);
    cout << "Connected to StreamServer" << endl;
  }
  catch(const XCF::GenericException& ex) {
    cerr << "Error at registering laserdata-xcf-subscriber"
	 << endl << "XCF::GenericException: " << ex.reason << endl;
    return -1;
  }
  return 0;
}


int xcf_navgoal_receive(char * location, size_t location_len,
			/** to be given back through xcf_navresult_send() */
			int * transaction_id,
			/** to be given back through xcf_navresult_send() */
			char * esv_state, size_t esv_state_len)
{
    if(0 != debugstream)
    {
	fprintf(debugstream, __FILE__ ": DEBUG xcf_navgoal_get()\n");
	strncpy(location, "Kitchen", location_len);
	*transaction_id = 1;
	strncpy(esv_state, "GoTo", esv_state_len);
	return 0;
    }
  
    string msg_status;
    try {
	// get message asynchronous from server
	msg_status = status_subscriber->receive(true);
    }
    catch (XCF::PublisherEmptyException) {
	//cerr << "publisher empty" << endl;
	return 1;
    }
    sd = extract<tStatusData>(Location(msg_status,"/MSG"));
    strncpy(location, sd.sData.c_str(), location_len);
    *transaction_id = 1;
    strncpy(esv_state, sd.sState.c_str(), esv_state_len);
    return 0;    
}

int xcf_navgoal_end()
{
  if(0 != debugstream){
    fprintf(debugstream, __FILE__ ": DEBUG xcf_navgoal_end()\n");
    return 0;
  }
  
  status_subscriber->destroy();
  return 0;
}

int xcf_navresult_init()
{
    if(0 != debugstream)
    {
	fprintf(debugstream, __FILE__ ": DEBUG xcf_navresult_init()\n");
	return 0;
    }
  
    // register new xcf-publishers
    try {
	_EQPublisher
	    = XCF::Publisher::create(DEFAULT_NAV2ESV_PUBLISHER_NAME);
	_EQPublisher->setSchema(string(SCHEMAPATH)+EQSCHEMA);
    }
    catch(const XCF::GenericException& ex) {
	cerr << "Error at registering xcf-publisher"
	     << endl << "XCF::GenericException: " << ex.reason << endl;
	return -1;
    }
    return 0;
}


int xcf_navresult_send(const char * result,
		       /** from xcf_navgoal_receive() */
		       int transaction_id,
		       /** from xcf_navgoal_receive() */
		       const char * esv_state)
{
    if(0 != debugstream){
	fprintf(debugstream, __FILE__ ": DEBUG xcf_navresult_set(%s, %d, %s)\n", result, transaction_id, esv_state);
	return 0;
    }
    
    tEventData evd;
    
    // calculate recent time
    timeval tCurTime;
    gettimeofday(&tCurTime, NULL);
    
    // converting time to ms since 1.1.1970
    uint64_t lMSecs;
    lMSecs  = static_cast<uint64_t>(tCurTime.tv_sec  * 1000);
    lMSecs += static_cast<uint64_t>(tCurTime.tv_usec / 1000);
    evd.lTimeStamp = lMSecs;
    evd.sGenerator = "NAV"; 
    evd.sOriginMod = "NAV";

    static int inum(0);
    ++inum;
    evd.iOriginNum = inum;

    evd.sRefMod = "DLG";
    evd.iRefNum = transaction_id;
    evd.sName = string(result);
    evd.sState = string(esv_state);
    evd.lBestBefore = lMSecs + 500;
    
    //converting struct to xml
    Location evdl(sEventData_tmpl,"/MSG");
    evdl = evd;

    //sending xml string
    try
    {
	_EQPublisher->send(evdl.getDocumentText());
    }
    catch(const XCF::ValidateFailedException& vex) 
    {
	cerr << "Error at XML Validataion"
	     << endl << "XCF::ValidateFailedException " << vex.reason << endl;
	cerr << evdl.getDocumentText() << endl;
	return -1;
    }
    return 0;
}

int xcf_navresult_end()
{
    if(0 != debugstream)
    {
	fprintf(debugstream, __FILE__ ": DEBUG xcf_navresult_end()\n");
	return 0;
    }
    _EQPublisher->destroy();
    return 0;
}


void xcf_hal_dryrun_on(FILE * stream)
{
    debugstream = stream;
}


void xcf_hal_dryrun_off()
{
    debugstream = 0;
}
