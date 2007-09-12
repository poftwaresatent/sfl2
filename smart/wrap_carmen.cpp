/* -*- mode: C++; tab-width: 2 -*- */
/* 
 * Copyright (C) 2007
 * Swiss Federal Institute of Technology, Zurich. All rights reserved.
 * 
 * Developed at the Autonomous Systems Lab.
 * Visit our homepage at http://www.asl.ethz.ch/
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

#include "wrap_carmen.hpp"
#include <sfl/api/Scanner.hpp>
#include <sfl/api/Scan.hpp>
#include <boost/shared_ptr.hpp>
#include <iostream>

#ifdef NPM_HAVE_CARMEN_ELROB
#include <carmen/carmen.h>
#include <smart_interface.h>
#include <axt_interface.h>
#include <smart_messages.h>
#endif


using namespace sfl;
using namespace boost;
using namespace std;


namespace wrap_carmen {
  
#ifdef NPM_HAVE_CARMEN_ELROB

	/* in order to handle the incoming carmen messages independently of the
		 requests from the nepumuk side, internal buffers are
		 needed. at the moment using static variables. better:
		 write a CarmenProxy that handles the communication 
		 and time stamping */

	/* NOTE: currently there is no error handling from 
		 the carmen framework.  Some functions like
		 carmen_ipc_initialize are handled internally by central */
	
	static elrob_smart_steering_message steering_msg;
	AXT_SCAN_STR alascaxt_scan;


	/* MESSAGE HANDLERS */
	void smart_ipc_smart_steering_handler(elrob_smart_steering_message* msg) {
		steering_msg.velocity=msg->velocity;
		steering_msg.steeringangle=msg->steeringangle;
		steering_msg.timestamp=msg->timestamp;
	}

	bool init_carmen(ostream *err){
	
		std::cout << "Initializing CARMEN Connection\n";
		char* tmpname = "nepumuk";
		carmen_ipc_initialize(1, &tmpname);
		return true;
	}

	bool cleanup_carmen(ostream *err)
	{

		carmen_ipc_disconnect();
		return true;
	}
	
  bool init_receive_steering(ostream * err)
  {
		
		elrob_smart_steering_subscribe_message(NULL, 
																					 (carmen_handler_t) smart_ipc_smart_steering_handler,
																					 CARMEN_SUBSCRIBE_LATEST);  
		return true;
  }
  

  
  bool receive_steering(double & velocity,
			double & steeringangle,
			ostream * err)
  {
		velocity=steering_msg.velocity;
		steeringangle=steering_msg.steeringangle;
    return true;
  }
  
  
  bool cleanup_receive_steering(ostream * err)
  {
		return true;
  }
  
  
  bool init_send_pos(ostream * err)
  {    

		elrob_smart_pos_define_message();

    return true;
  }
  
  
  bool send_pos(double x, double y, double theta,
		double velocity, double steering,
		ostream * err)
  {
    elrob_smart_pos_message msg;
    msg.pose.x = x;
    msg.pose.y = y;
    msg.pose.z = 0;
    msg.pose.phi = 0;		/**< roll **/
    msg.pose.theta = 0;		/**< pitch **/
    msg.pose.psi = theta;	/**< yaw **/
    msg.velocity = velocity;
    msg.steering_angle = steering;
    msg.timestamp = -1;		// this should probably change each tick

		char host[8]="nepumuk";
		msg.host=host;
    
		//currently only some of the data is supported by the elrob carmen framework
		//(should be upgraded)

		elrob_smart_pos_send_message(msg.pose.x, 
																 msg.pose.y, 
																 msg.pose.z,
																 msg.pose.phi,
																 msg.pose.theta,
																 msg.pose.psi,
																 msg.velocity,
																 msg.steering_angle,
																 msg.timestamp);

    return true;
  }
  
  
  bool cleanup_send_pos(ostream * err)
  {
    return true;
  }
  
  
  bool init_send_laser(ostream * err)
  {
		axt_define_message();

    return true;
  }
  
  
  bool send_laser(const Scanner & scanner,
		  ostream * err)
  {

		shared_ptr<Scan> scan(scanner.GetScanCopy());
		
		//ALASCAXT based message is currently sent

		//currently not handled values
		alascaxt_scan.header.version=0;
		alascaxt_scan.header.scanner_type=0;
		alascaxt_scan.header.ecu_id=0;
		alascaxt_scan.header.time_stamp=-1;
		alascaxt_scan.header.timestamp_sinc=-1;

		alascaxt_scan.header.start_angle=scanner.phi0;
		alascaxt_scan.header.end_angle=scanner.phi0+scanner.phirange;
		alascaxt_scan.header.scan_counter=scan->data.size();

		int num_valid_pts(0);
		for(size_t ii(0); ii < scan->data.size() && ii<AXT_MAX_SCAN_POINTS; ++ii){
			if(scan->data[ii].in_range){
				alascaxt_scan.points[ii].point_status=AXT_PT_STATUS_OK;
				alascaxt_scan.points[ii].x=scan->data[ii].globx;
				alascaxt_scan.points[ii].y=scan->data[ii].globy;
				//for z is a hard coded stuff to appear as a "big obstacle"
				//because nepumuk currenly is a 2D simulation
				alascaxt_scan.points[ii].z=100; 

				/*
				fprintf(stderr, "[%d]: x=%f, y=%f, locx=%f, locy=%f, rho=%f, phi=%f\n",
								ii, scan->data[ii].globx, scan->data[ii].globy,
								scan->data[ii].locx, scan->data[ii].locy, scan->data[ii].rho, scan->data[ii].phi);
				*/				
			}else{
				alascaxt_scan.points[ii].point_status=AXT_PT_STATUS_INVALID;
			}
				num_valid_pts++;
		}
		
		alascaxt_scan.header.num_points=num_valid_pts;
		
		axt_send_message(&alascaxt_scan);
		
		return true;
  }
  
  
  bool cleanup_send_laser(ostream * err)
  {
    return true;
  }



#else  
# warning Using dummy implementations of Carmen communication.
  

	bool init_carmen(ostream *err){		
		*err << "dummy implementation\n";
		return true;
	}

	bool cleanup_carmen(ostream *err){
		*err << "dummy implementation\n";
		return true;
	}
	
  bool init_receive_steering(ostream * err)
  {
		*err << "dummy implementation\n";
		return true;
  }
  

  
  bool receive_steering(double & velocity,
			double & steeringangle,
			ostream * err)
  {
		*err << "dummy implementation\n";
    return true;
  }
  
  
  bool cleanup_receive_steering(ostream * err)
  {
		*err << "dummy implementation\n";
		return true;
  }
  
  
  bool init_send_pos(ostream * err)
  {    
		*err << "dummy implementation\n";
    return true;
  }
  
  
  bool send_pos(double x, double y, double theta,
		double velocity, double steering,
		ostream * err)
  {
		*err << "dummy implementation\n";
		return true;
  }
  
  
  bool cleanup_send_pos(ostream * err)
  {
		*err << "dummy implementation\n";
    return true;
  }
  
  
  bool init_send_laser(ostream * err)
  {
		*err << "dummy implementation\n";
    return true;
  }
  
  
  bool send_laser(const Scanner & scanner,
		  ostream * err)
  {
		*err << "dummy implementation\n";
    return true;
  }
  
  
  bool cleanup_send_laser(ostream * err)
  {
		*err << "dummy implementation\n";
    return true;
  }

#endif // NPM_HAVE_CARMEN_ELROB
  
}
