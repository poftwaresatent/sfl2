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


#include "expo_parameters.h"
#include <cmath>


void expo_default_parameters(struct expo_parameters * params)
{
  params->front_channel     = 0;
  params->front_nscans      = 181;
  params->front_mount_x     = 0.15;
  params->front_mount_y     = 0;
  params->front_mount_theta = 0;
  params->front_rhomax      = 8;
  params->front_phi0        = -M_PI/2;
  params->front_phirange    = M_PI;
  
  params->rear_channel      = 1;
  params->rear_nscans       = 181;
  params->rear_mount_x      = -0.15;
  params->rear_mount_y      = 0;
  params->rear_mount_theta  = M_PI;
  params->rear_rhomax       = 8;
  params->rear_phi0         = -M_PI/2;
  params->rear_phirange     = M_PI;
  
  params->model_security_distance = 0.02;
  params->model_wheelbase         = 0.521;
  params->model_wheelradius       = 0.088;
  params->model_qd_max            = 6.5;
  params->model_qdd_max           = 6.5;
  params->model_sd_max            = 0.5;
  params->model_thetad_max        = 2.0;
  params->model_sdd_max =
    0.75 * params->model_wheelradius * params->model_qd_max;
  params->model_thetadd_max =
    1.5 * params->model_wheelradius * params->model_qd_max
    / params->model_wheelbase;
  
  params->dwa_dimension       = 41;
  params->dwa_grid_width      = 2.3;
  params->dwa_grid_height     = 1.7;
  params->dwa_grid_resolution = 0.03;
  params->dwa_alpha_distance  = 0.6;
  params->dwa_alpha_heading   = 0.3;
  params->dwa_alpha_speed     = 0.1;
  
  params->bband_enabled           = 1;
  params->bband_shortpath         = 2;
  params->bband_longpath          = 10;
  params->bband_maxignoredistance = 4;
  
  params->mp_strict_dwa         = 0;
  params->mp_auto_adapt_dwa     = 0;
  params->mp_dtheta_starthoming = 10 * M_PI / 180;
  params->mp_dtheta_startaiming = 45 * M_PI / 180;
}
