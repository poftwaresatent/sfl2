/* -*- mode: C++; tab-width: 4; c-basic-offset: 4; c-file-offsets: ((substatement-open . 0)) -*- */
/**
 * @file     esvdata.h
 * @author   Thorsten Spexard
 * @date     24.08.2005
 * @brief    defining and converting esv data in C++ stucts and XML
 */

#ifndef ESVDATA_CPP
#define ESVDATA_CPP

#include <string>
#include <xcf/xcf.hpp>
#include <xmltio/xmltio.hpp>

#define NAV_STATUS_COMPLETED "COMPLETED"
#define NAV_STATUS_PLANNING  "PLANNING"
#define NAV_STATUS_DRIVING   "DRIVING"

using namespace std;
using namespace XCF;
using namespace xmltio;


/**
 * xml structure for status data
 */
const std::string sStatusData_tmpl = (string)
  "<?xml version=\"1.0\"?>"
+   "<MSG xmlns:xs=\"http://www.w3.org/2001/XMLSchema-instance\" xs:type=\"status\">"
+   "<GENERATOR/>"
+   "<TIMESTAMP/>"
+   "<ID>"
+     "<ORIGIN mod=\"\"/>"
+   "</ID>"
+   "<NAME/>"
+   "<STATE/>"
+ "</MSG>";

/**
 * xml structure for order data
 */
const std::string sOrderData_tmpl = (string)
  "<?xml version=\"1.0\"?>"
+   "<MSG xmlns:xs=\"http://www.w3.org/2001/XMLSchema-instance\" xs:type=\"order\">"
+   "<GENERATOR/>"
+   "<TIMESTAMP/>"
+   "<ID>"
+     "<ORIGIN mod=\"\"/>"
+   "</ID>"
+   "<NAME/>"
+   "<STATE/>"
+ "</MSG>";

/**
 * xml structure for reject data
 */
const std::string sRejectData_tmpl = (string)
  "<?xml version=\"1.0\"?>"
+   "<MSG xmlns:xs=\"http://www.w3.org/2001/XMLSchema-instance\" xs:type=\"reject\">"
+   "<GENERATOR/>"
+   "<TIMESTAMP/>"
+   "<ID>"
+     "<ORIGIN mod=\"\"/>"
//+     "<REF mod=\"\"/>"          //Optional
+   "</ID>"
+   "<NAME/>"
+   "<STATE/>"
+   "<INITTIME/>"
+   "<EVENT/>"
+ "</MSG>";

/**
 * xml structure for event data
 */
const std::string sEventData_tmpl = (string)
  "<?xml version=\"1.0\"?>"
+   "<MSG xmlns:xs=\"http://www.w3.org/2001/XMLSchema-instance\" xs:type=\"event\">"
+   "<GENERATOR/>"
+   "<TIMESTAMP/>"
+   "<ID>"
+     "<ORIGIN mod=\"\"/>"
//+     "<REF mod=\"\"/>"          //Optional
+   "</ID>"
+   "<NAME/>"
+   "<STATE/>"
+   "<BESTBEFORE/>"
+ "</MSG>";

/**
 * C++ struct for status data
 */
struct tStatusData {
	/**
	 * initialising struct data
	 */
	tStatusData()
	{
		lTimeStamp = 0;
		iOriginNum = 0;
	}
	
    /**
     * generator of the message, abbreviated to three letters (e.g. ESV)
     */
    string sGenerator;  

    /**
     * milliseconds since 1970
     */
    uint64_t lTimeStamp; 

    /**
     * module which caused the ESV to send this status message, 
     * abbreviated to three letters (e.g. DLG)
     */
    string sOriginMod;  

    /**
     * reference number of the event sent by the origin module
     */
    int iOriginNum;  
    
    /**
     * suggested action for the receiver
     */
    string sName;

    /**
     * recent state of the ESV
     */
    string sState;

    /**
     * content dependend data
     */
    string sData;
};

/**
 * C++ struct for reject data
 */
struct tRejectData {
	/**
	 * initialising struct data
	 */
	tRejectData()
	{
		lTimeStamp = 0;
		iOriginNum = 0;
		iRefNum    = 0;
		lInitTime  = 0;
	}
    /**
     * generator of the message, abbreviated to three letters (e.g. ESV)
     */
    string sGenerator;  

    /**
     * milliseconds since 1970
     */
    uint64_t lTimeStamp; 

    /**
     * module which originally activated the module of which the answere is 
	 * rejected, abbreviated to three letters (e.g. ESV)
     */
    string sOriginMod;  

    /**
     * module of which the answere is rejected, 
     * abbreviated to three letters (e.g. DLG)
     */
    string sRefMod;  

    /**
     * reference number of the event sent by the origin module
     */
    int iOriginNum;  

    /**
     * reference number of the event sent by the reference module
     */
    int iRefNum;  
    
    /**
     * reason for rejection
     */
    string sName;

    /**
     * recent state of the ESV
     */
    string sState;

	/**
     * time when the last event took place, in milliseconds since 1970
     */
    uint64_t lInitTime;

    /**
     * event not processed, causing the rejection
     */
    string sEventName;
};

/**
 * C++ struct for order data
 */
struct tOrderData {
	/**
	 * initialising struct data
	 */
	tOrderData()
	{
		lTimeStamp = 0;
		iOriginNum = 0;
	}

    /**
     * generator of the message, abbreviated to three letters (e.g. ESV)
     */
    string sGenerator;  

    /**
     * milliseconds since 1970
     */
    uint64_t lTimeStamp; 

    /**
     * module which caused the ESV to send this status message, 
     * abbreviated to three letters (e.g. DLG)
     */
    string sOriginMod;  

    /**
     * reference number of the event sent by the origin module
     */
    int iOriginNum;  
    
    /**
     * suggested action for the receiver
     */
    string sName;

    /**
     * recent state of the ESV
     */
    string sState;

    /**
     * general content dependend data
     */
    string sData;

	/**
	 * ROR specific data
	 */
	string sGestExp;
	string sObjType;
	string sObjColor;
	string sObjIndef;
	string sObjDef;
};

/**
 * C++ struct for event data
 */
struct tEventData {
	/**
	 * initialising struct data
	 */
	tEventData()
	{
		lTimeStamp     = 0;
		iOriginNum     = 0;
		iRefNum        = 0;
		lBestBefore    = 0;
		iCPid          = 0;
		iObjectListNum = 0;
	    lObjTime       = 0;
	    iObjID         = 0;
	    fObjScore      = 0.;
	}

    /**
     * generator of the message, abbreviated to three letters (e.g. ESV)
     */
    string sGenerator;  

    /**
     * milliseconds since 1970
     */
    uint64_t lTimeStamp; 

    /**
     * module which originally activated the module of which the answere is 
	 * rejected, abbreviated to three letters (e.g. ESV)
     */
    string sOriginMod;  

    /**
     * module of which the answere is rejected, 
     * abbreviated to three letters (e.g. DLG)
     */
    string sRefMod;  

    /**
     * reference number of the event sent by the origin module
     */
    int iOriginNum;  

    /**
     * reference number of the event sent by the reference module
     */
    int iRefNum;  

    /**
     * suggested action for the receiver
     */
    string sName;

    /**
     * recent state of the ESV
     */
    string sState;

    /**
     * content dependend data
     */
    string sData;

    /**
     * expiring date of message [milliseconds since 1970]
     */
    uint64_t lBestBefore; 

	/**
	 * PTA specific data
	 */
	int iCPid;
	string sCPname;

	/**
	 * ROR specific data
	 */
	string sGestEventName, sGestEventData, sGesEventSrcmod;
	string sGestCondName, sGestCondTgtmod;
	string sGestCondDataName;
	string sGestDataExistence;

	int iObjectListNum;
	string sFailureCause;
	string sObjType, sObjIndef, sObjDef;
	string sObjColor;
	uint64_t lObjTime;
	int iObjID;
	float fObjScore;
};


/**
 * automatic convert method for struct -> xml,
 * called every time a data struct (d) is assigned to a xml location (l),
 * like 'l = d;'
 */
void convert(const tStatusData& std, Location& loc);

/**
 * automatic convert method for struct -> xml,
 * called every time a data struct (d) is assigned to a xml location (l),
 * like 'l = d;'
 */
void convert(const tRejectData& d, Location& loc);

/**
 * automatic convert method for struct -> xml,
 * called every time a data struct (d) is assigned to a xml location (l),
 * like 'l = d;'
 */
void convert(const tEventData& d, Location& loc);

/**
 * automatic convert method for struct -> xml,
 * called every time a data struct (d) is assigned to a xml location (l),
 * like 'l = d;'
 */
void convert(const tOrderData& d, Location& loc);





/**
 * extracts data from xml (msg) to data struct (d) 
 * e.g. d = extract<tData>(Location(msg,"/MSG"));
 */
template<>
tStatusData extract<tStatusData>::get() const;

/**
 * extracts data from xml (msg) to data struct (d) 
 * e.g. d = extract<tData>(Location(msg,"/MSG"));
 */
template<>
tRejectData extract<tRejectData>::get() const;

/**
 * extracts data from xml (msg) to data struct (d) 
 * e.g. d = extract<tData>(Location(msg,"/MSG"));
 */
template<>
tEventData extract<tEventData>::get() const;

/**
 * extracts data from xml (msg) to data struct (d) 
 * e.g. d = extract<tData>(Location(msg,"/MSG"));
 */
template<>
tOrderData extract<tOrderData>::get() const;

#endif
