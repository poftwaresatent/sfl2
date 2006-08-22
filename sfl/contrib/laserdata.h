/* -*- mode: C++; tab-width: 4; c-basic-offset: 4; c-file-offsets: ((substatement-open . 0)) -*- */
/**
 * @file     laserdata.h
 * @author   Erik Weitnauer, Thorsten Spexard
 * @date     15.08.2005
 * @brief    defining and converting laser data in C++ stucts and XML
 */

#ifndef LASERDATA_CPP
#define LASERDATA_CPP

#include <string>
#include <xcf/xcf.hpp>
#include <xmltio/xmltio.hpp>

using namespace std;
using namespace XCF;
using namespace xmltio;

/**
 * resolution of laser range finder, in common 361 points
 */
#define LASER_POINTS 361

/**
 * xml structure for laserdata
 */

const std::string sLaserData_tmpl = (string)
  "<?xml version=\"1.0\"?>"
+ "<MSG xmlns:xs=\"http://www.w3.org/2001/XMLSchema-instance\" xs:type=\"laser\">"
+   "<GENERATOR/>"
+   "<TIMESTAMP/>"
+   "<MEASUREMENTS num=\"\"/>"
+ "</MSG>";

/**
 * C++ struct for laser data
 */
struct tLaserData {
    /**
     * seconds since 1970
     * depricated
     */
    double dTimeStamp_Sec;
    
    /**
     * remaining microseconds, according to seconds since 1970
     * deprecated
     */
    double dTimeStamp_uSec;      
    
    /**
     * generator of the message, abbreviated to three letters (e.g. RDS)
     */
    string sGenerator;        

    /**
     * milliseconds since 1970
     */
    uint64_t lTimeStamp;     

    /**
     * resolution of laser range finder, in common 361 points
     */
    int    iLaserPoints;    

    /**
     * array with distance measurements generated by laser range finder,
     * distances in mm relative to the robot
     */
    float fPoints[LASER_POINTS];   
};

/**
 * automatic convert method for struct -> xml,
 * called every time a struct (ld) is assigned to a location (ll),
 * like 'll = ld;'
 */
void convert(const tLaserData& ld, Location& loc);

/**
 * extracts laser data from xml (msg_laser) to tLaserData (ld) struct
 * e.g. ld = extract<tLaserData>(Location(msg_laser,"/MSG"));
 */
template<>
tLaserData extract<tLaserData>::get() const;

#endif