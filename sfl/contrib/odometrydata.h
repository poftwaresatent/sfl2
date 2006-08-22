/* -*- mode: C++; tab-width: 4; c-basic-offset: 4; c-file-offsets: ((substatement-open . 0)) -*- */
/**
 * @file     odometrydata.h
 * @author   Thorsten Spexard
 * @date     15.08.2005
 * @brief    defining and converting odometry data in C++ stucts and XML
 */

#ifndef ODOMETRYDATA_CPP
#define ODOMETRYDATA_CPP

#include <string>
#include <xcf/xcf.hpp>
#include <xmltio/xmltio.hpp>

using namespace std;
using namespace XCF;
using namespace xmltio;

/**
 * xml structure for odometry data
 */
const std::string sOdometryData_tmpl = (string)
  "<?xml version=\"1.0\"?>"
+ "<MSG xmlns:xs=\"http://www.w3.org/2001/XMLSchema-instance\" xs:type=\"odometry\">"
+   "<GENERATOR/>"
+   "<TIMESTAMP/>"
+   "<POSITION>"
+     "<LOCATION>"
+       "<CARTESIAN x=\"\" y=\"\"/>"
+     "</LOCATION>"
+     "<ORIENTATION>"
+       "<YAW rad=\"\"/>"
+     "</ORIENTATION>"
+   "</POSITION>"
+ "</MSG>";

/**
 * C++ struct for odometry data
 */
struct tOdometryData {
    /**
     * milliseconds since 1970
     */
    uint64_t lTimeStamp;

    /**
     * generator of the message, abbreviated to three letters (e.g. RDS)
     */
    string sGenerator;  

    /**
     * x coordinate relative to starting position [m],
     * at orientation = 0, x axis is in forward direction
     */
    float fXCoord;                // m

    /**
     * y coordinate relative to starting position [m],
     * perpendicular-left to the positive x axis
     */
    float fYCoord;                // m

    /**
     * orientatiob relative to starting orientation [rad],
     * range is [-PI .. +PI], counter-clockwise
     */
    float fYaw;                   // rad
};

/**
 * automatic convert method for struct -> xml,
 * called every time a struct (od) is assigned to a location (ol),
 * like 'ol = od;'
 */
void convert(const tOdometryData& odo, Location& loc);

/**
 * extracts odometry data from xml (msg_odo) to tOdometryData (od) struct
 * e.g. od = extract<tOdometryData>(Location(msg_odo,"/MSG"));
 */
template<>
tOdometryData extract<tOdometryData>::get() const;

#endif
