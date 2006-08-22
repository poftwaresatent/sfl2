/* -*- mode: C++; tab-width: 4; c-basic-offset: 4; c-file-offsets: ((substatement-open . 0)) -*- */
/**
 * @file     AttTarget.h
 * @author   Sebastian Lang, Thorsten Spexard
 * @date     30.09.2003
 * @brief    target position for sensors and robot base
 */

#ifndef _ATTTARGET_H
#define _ATTTARGET_H

// Header for Log4cpp using the global xcflogger
#include "xcf/log4cpp/XCFAppender.hpp"
//#include "xcf/util/Debug.hpp"      // nur fuer Debugging

#include <xcf/xcf.hpp>
#include <xmltio/xmltio.hpp>

using namespace std;
using namespace XCF;
using namespace xmltio;

/**
 * velocity steps for movement
 */
typedef enum e_AttSpeed {AS_NO, AS_SLOW, AS_MEDIUM, AS_FAST};

//  Zielposition für Sensorik.
//  
//  Die Aufmerksamkeits-Steuerung sorgt dafür, dass die Sensorik des Roboters so
//  ausgerichtet wird, dass möglichst viel wichtige Information gewonnen werden
//  kann.  Die Ansteuerung der Sensorik wird hier durch Zielpositionen im Raum
//  (Zylinderkoordinaten) beschrieben.  Gleichzeitig wird angegeben, mit welcher
//  Geschwindigkeit(sstufe) die Zielposition angefahren werden soll.
//  

/**
 * Target position relative to robot for sensors and robot base:
 * Sensors and base are controlled by target positions using cylindric coordinates.
 * Additionally a velocity step is given for the movement to reach the target position.
 */
class t_AttTarget
{
  private:
	/**
	 * flag indicates wether the target position is valid,
	 * true if necessary data was set and distances are > 0
	 */
	bool       m_bValid; 

	/**
	 * target position angle in rad
	 */
	float      m_fPosAng;

	/**
	 * target position distance in mm
	 */
	float      m_fPosDst;     ///< Zielposition: Abstand

	/**
	 * target position height from ground in mm
	 */
	float      m_fPosHgt;

	/**
	 * target zoom value
	 */
	float      m_fPosZoom;

	/**
	 * rotation speed
	 */
	e_AttSpeed m_eSpeedRot;
	
	/**
	 * translation speed
	 */
	e_AttSpeed m_eSpeedTrans;

  public:
	/**
	* constructor
	*/
	t_AttTarget();

	/**
	* destructor
	*/
	virtual ~t_AttTarget() {vFinit();}
	
  private:
	void vInit();
	void vFinit() {}
	
  public:
	/**
	 * Indicates if a target was set by vSetPos.
	 */
	bool bIsValid() const {return m_bValid;}

	/**
	 * Setting a target position in cylindric coordinates [rad, mm, mm].
	 * The zoom value is valid within the interval [0..1,0]
	 */
	void vSetPos(float fAng, float fDst, float fHgt, float fZoom=0.0);

	/**
	 * Setting qualitative speeds for rotation and translation.
	 * Possible are AS_NO, AS_SLOW, AS_MEDIUM and AS_FAST.
	 */
	void vSetSpeed(e_AttSpeed eRot, e_AttSpeed eTrans = AS_NO);

	/**
	 * Method returns the target angle [rad].
	 */
	float fGetPosAng() const {return m_fPosAng;}

	/**
	 * Method returns the target distance [mm].
	 */
	float fGetPosDst() const {return m_fPosDst;}

	/**
	 * Method returns the target height [mm].
	 */
	float fGetPosHgt() const {return m_fPosHgt;}

	/**
	 * Method returns the target zoom value [0..1,0].
	 */
	float fGetPosZoom() const {return m_fPosZoom;}

	/**
	 * Method returns the qualitative target rotation speed [rad/s].
	 */
	e_AttSpeed eGetSpeedRot() const {return m_eSpeedRot;}

	/**
	 * Method returns the qualitative target translation speed [mm/s].
	 */
	e_AttSpeed eGetSpeedTrans() const {return m_eSpeedTrans;}

	/**
	 * Print out target information.
	 */
	void vInfo() const;

	/**
	 * automatic convert method for struct -> xml,
	 * called every time a struct (attd) is assigned to a location (attl),
	 * like 'attl = attd;'
	 */
	static void convert(const t_AttTarget& attTarget, Location& loc);

	/**
	 * extracts data from xml, called by template<> t_AttTarget extract
	 */
	void extractFrom(const Location& loc_);
};

/**
 * This method is automaticalliy called if a C++ struct is assigned to a location (loc = struct;).
 * It wraps t_AttTarget::convert
 */
void convert(const t_AttTarget& attTarget, Location& loc);

/**
 * Template to extract movement data from xml to the according C++ struct.
 * E.g. struct = extract<t_AttTarget>(Location(xmlstring,"/root"));
 */
template<>
t_AttTarget extract<t_AttTarget>::get() const;

#endif /* _ATTTARGET_H */
