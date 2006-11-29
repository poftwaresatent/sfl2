#include "expoparams.hpp"
#include <npm/common/util.hpp>

using namespace npm;
using namespace boost;

expoparams::
expoparams(shared_ptr<RobotDescriptor> descriptor)
{
  expo_default_parameters(this);
  if( ! descriptor)
    return;
  
  string_to(descriptor->GetOption("front_channel"), front_channel);
  string_to(descriptor->GetOption("front_nscans"), front_nscans);
  string_to(descriptor->GetOption("front_mount_x"), front_mount_x);
  string_to(descriptor->GetOption("front_mount_y"), front_mount_y);
  string_to(descriptor->GetOption("front_mount_theta"), front_mount_theta);
  string_to(descriptor->GetOption("front_rhomax"), front_rhomax);
  string_to(descriptor->GetOption("front_phi0"), front_phi0);
  string_to(descriptor->GetOption("front_phirange"), front_phirange);
  
  string_to(descriptor->GetOption("rear_channel"), rear_channel);
  string_to(descriptor->GetOption("rear_nscans"), rear_nscans);
  string_to(descriptor->GetOption("rear_mount_x"), rear_mount_x);
  string_to(descriptor->GetOption("rear_mount_y"), rear_mount_y);
  string_to(descriptor->GetOption("rear_mount_theta"), rear_mount_theta);
  string_to(descriptor->GetOption("rear_rhomax"), rear_rhomax);
  string_to(descriptor->GetOption("rear_phi0"), rear_phi0);
  string_to(descriptor->GetOption("rear_phirange"), rear_phirange);
  
  string_to(descriptor->GetOption("model_security_distance"),
	    model_security_distance);
  string_to(descriptor->GetOption("model_wheelbase"), model_wheelbase);
  string_to(descriptor->GetOption("model_wheelradius"), model_wheelradius);
  string_to(descriptor->GetOption("model_axlewidth"), model_axlewidth);
  string_to(descriptor->GetOption("model_qd_max"), model_qd_max);
  string_to(descriptor->GetOption("model_qdd_max"), model_qdd_max);
  string_to(descriptor->GetOption("model_sd_max"), model_sd_max);
  string_to(descriptor->GetOption("model_thetad_max"), model_thetad_max);
  string_to(descriptor->GetOption("model_sdd_max"), model_sdd_max);
  string_to(descriptor->GetOption("model_thetadd_max"), model_thetadd_max);
  
  string_to(descriptor->GetOption("dwa_dimension"), dwa_dimension);
  string_to(descriptor->GetOption("dwa_grid_width"), dwa_grid_width);
  string_to(descriptor->GetOption("dwa_grid_height"), dwa_grid_height);
  string_to(descriptor->GetOption("dwa_grid_resolution"), dwa_grid_resolution);
  string_to(descriptor->GetOption("dwa_alpha_distance"), dwa_alpha_distance);
  string_to(descriptor->GetOption("dwa_alpha_heading"), dwa_alpha_heading);
  string_to(descriptor->GetOption("dwa_alpha_speed"), dwa_alpha_speed);
  
  string_to_bool(descriptor->GetOption("bband_enabled"), bband_enabled);
  string_to(descriptor->GetOption("bband_shortpath"), bband_shortpath);
  string_to(descriptor->GetOption("bband_longpath"), bband_longpath);
  string_to(descriptor->GetOption("bband_maxignoredistance"),
	    bband_maxignoredistance);
  
  string_to_bool(descriptor->GetOption("mp_strict_dwa"), mp_strict_dwa);
  string_to_bool(descriptor->GetOption("mp_auto_adapt_dwa"),
		 mp_auto_adapt_dwa);
  string_to(descriptor->GetOption("mp_dtheta_starthoming"),
	    mp_dtheta_starthoming);
  string_to(descriptor->GetOption("mp_dtheta_startaiming"),
	    mp_dtheta_startaiming);
}
