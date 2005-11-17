#ifndef XCF_HAL_H
#define XCF_HAL_H

#include <sys/time.h>
#include <iostream>
#include <xcf/xcf.hpp>
#include <xmltio/xmltio.hpp>

#include <laserdata.h>
#include <odometrydata.h>
#include <movementdata.h>

/**
 * schema
 */
#define MOVESCHEMA            "MoveMsgKE1.xsd"
#define SCHEMAPATH           "../cfg/schemas/"

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


#ifdef __cplusplus
extern "C" {
#endif // __cplusplus

using namespace std;
using namespace XCF;
using namespace xmltio;


    static XCF::SubscriberPtr odo_subscriber;
    static tOdometryData odo;

    int xcf_odometry_init()
    {
	// register xcf-subscriber and connect to xcf-publisher
	try {
	    odo_subscriber = XCF::Subscriber::create(DEFAULT_ODOMETRY_PUBLISHER_NAME);
	    // no message cueing
	    odo_subscriber->setOnlyReceiveLast(true);
	    cout << "Connected to StreamServer" << endl;
	}
	catch(const XCF::GenericException& ex) {
	    cerr << "Error at registering odometry-xcf-subscriber"
		 << endl << "XCF::GenericException: " << ex.reason << endl;
	}
	return 0;
    }
    
    int xcf_odometry_get(double * x, double * y, double * theta_rad,
			 uint64_t * timestamp_ms)
    {
	string msg_odo;
	try {
	    // get message asynchronous from server
	    msg_odo = odo_subscriber->receive(true);
	}
	catch (XCF::PublisherEmptyException) {
	    //cerr << "publisher empty" << endl;
	    *x = (double) odo.fXCoord;
	    *y = (double) odo.fYCoord;
	    *theta_rad = (double) odo.fYaw;
	    *timestamp_ms = odo.lTimeStamp;
	    return 0;
	}
	odo = extract<tOdometryData>(Location(msg_odo,"/MSG"));
	*x = (double) odo.fXCoord;
	*y = (double) odo.fYCoord;
	*theta_rad = (double) odo.fYaw;
	*timestamp_ms = odo.lTimeStamp;
	return 0;
    }
    
    int xcf_odometry_end()
    {
	odo_subscriber->destroy();
	return 0;
    }
    

    static XCF::PublisherPtr _RelPosPublisher;
    int xcf_speed_init()
    {
	// register new xcf-publishers
	try {
	    _RelPosPublisher = XCF::Publisher::create(DEFAULT_REL_POS_PUBLISHER_NAME);
	    _RelPosPublisher->setSchema(string(SCHEMAPATH)+MOVESCHEMA);
	}
	catch(const XCF::GenericException& ex) {
	    cerr << "Error at registering xcf-publisher"
		 << endl << "XCF::GenericException: " << ex.reason << endl;
	}
	return 0;
    }
    
    int xcf_speed_set(double v, double w)
    {
	t_MovementData mvd;
	
	// calculate recent time
	timeval tCurTime;
	gettimeofday(&tCurTime, NULL);
	
	// converting time to ms since 1.1.1970
	uint64_t lMSecs;
	lMSecs  = (uint64_t) tCurTime.tv_sec  * 1000;
	lMSecs += (uint64_t) tCurTime.tv_usec / 1000;
	mvd.lTimeStamp = lMSecs;
	mvd.dVTrans = v;
	mvd.dVRot = w;

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
	}
	return 0;
    }
    
    int xcf_speed_end()
    {
	_RelPosPublisher->destroy();
	return 0;
    }


    static XCF::SubscriberPtr laser_subscriber;
    static tLaserData ld;

    int xcf_scan_init(int channel)
    {
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
	}
	return 0;
    }
    
    int xcf_scan_get(int channel, double * rho, int rho_len,
		     uint64_t * timestamp_ms)
    {
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
	laser_subscriber->destroy();
	return 0;
    }
  
#ifdef __cplusplus
}
#endif // __cplusplus
  
#endif // XCF_HAL_H
