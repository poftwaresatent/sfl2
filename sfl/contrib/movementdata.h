/* -*- mode: C++; tab-width: 4; c-basic-offset: 4; c-file-offsets: ((substatement-open . 0)) -*- */
/**
 * @file     movementdata.h
 * @author   Thorsten Spexard
 * @date     08.08.2005
 * @brief    Converting and extracting movement data to and from xml-structure
 */

#ifndef MOVEMENTDATA_CPP
#define MOVEMENTDATA_CPP

#include <string>
#include <xcf/xcf.hpp>
#include <xmltio/xmltio.hpp>
#include "AttTarget.h"

using namespace std;
using namespace XCF;
using namespace xmltio;

/**
 * xml structure for movement data
 */
const std::string sMovementData_tmpl = (string)
 "<?xml version=\"1.0\"?>"
+"<MSG xmlns:xs=\"http://www.w3.org/2001/XMLSchema-instance\" xs:type=\"movement\">"
+  "<GENERATOR/>"
+  "<TIMESTAMP/>"
+  "<BASE>"
+    "<LOCATION>"
+      "<POLAR dist=\"\">"
+         "<AZIMUTH rad=\"\"/>"
+      "</POLAR>"
+    "</LOCATION>"
+    "<ROTVEL qual=\"\"/>"
+    "<TRANSVEL qual=\"\"/>"
+  "</BASE>"
+  "<CAMERA>"
+    "<LOCATION>"
+      "<CYLINDRIC dist=\"\" height=\"\">"
+        "<AZIMUTH rad=\"\"/>"
+      "</CYLINDRIC>"
+    "</LOCATION>"
+    "<ROTVEL qual=\"\"/>"
+    "<ZOOM quant=\"\"/>"
+  "</CAMERA>"
//+  "<TRACKINGSTATE/>"  //optional
//+  "<DRIVE>"             //optional
//+    "<ROTVEL quant=\"\"/>"
//+    "<TRANSVEL quant=\"\"/>"
//+  "</DRIVE>"
+"</MSG>";

/**
 * C++ structure which is converted to / extracted from xml-data
 */
struct t_MovementData {
	t_MovementData(){
		iTrackingState = 0; 
		dVTrans = 0.0;
		dVRot = 0.0;
	}

	/**
	 * milliseconds since 1970
	 */
    uint64_t lTimeStamp;           

	/**
	 * generating module, e.g. PTA, NAV, ...
	 */
    string sGenerator;

	/**
	 * base target
	 */
    t_AttTarget tBase;
	
	/**
	 * camera target
	 */
    t_AttTarget tCamera;
	
	/**
	 * tracking state (0:lost, 1:anchored, 2:grounded)
	 */
    int iTrackingState; 

	/**
	 * absolute translation velocity in meters/s
	 */
	double dVTrans;

	/**
	 * absolute rotation velocity in rad/s
	 */
	double dVRot;
};

/**
 * This method is automaticalliy called if a C++ struct is assigned to a location (loc = struct;).
 */
void convert(const t_MovementData& esd, Location& loc);

/**
 * Template to extract movement data from xml to the according C++ struct.
 * E.g. struct = extract<t_MovementData>(Location(xmlstring,"/root"));
 */
template<>
t_MovementData extract<t_MovementData>::get() const;

/**
 * Converts the qualitive movemodes represented as strings in xml to enumeration type e_AttSpeed.
 */
e_AttSpeed eMoveMode2Int(string str);
#endif
