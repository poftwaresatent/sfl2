/* 
 * Copyright (C) 2006
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


#ifndef EXPO_PARAMETERS_H
#define EXPO_PARAMETERS_H

#ifdef __cplusplus
extern "C" {
#endif // __cplusplus  
  
  struct expo_parameters
  {
    int front_channel, front_nscans;
    double front_mount_x, front_mount_y, front_mount_theta;
    double front_rhomax, front_phi0, front_phirange;
    
    int rear_channel, rear_nscans;
    double rear_mount_x, rear_mount_y, rear_mount_theta;
    double rear_rhomax, rear_phi0, rear_phirange;
    
    double model_security_distance, model_wheelbase, model_wheelradius;
    double model_qd_max, model_qdd_max, model_sd_max, model_thetad_max;
    double model_sdd_max, model_thetadd_max;  
    
    int dwa_dimension;
    double dwa_grid_width, dwa_grid_height, dwa_grid_resolution;
    double dwa_alpha_distance, dwa_alpha_heading, dwa_alpha_speed;
    
    int bband_enabled;
    double bband_shortpath, bband_longpath, bband_maxignoredistance;
    
    int mp_strict_dwa, mp_auto_adapt_dwa;
    double mp_dtheta_starthoming, mp_dtheta_startaiming;
  };
  
  void expo_default_parameters(struct expo_parameters * params);
  
#ifdef __cplusplus
}
#endif // __cplusplus  

#endif // EXPO_PARAMETERS_H
