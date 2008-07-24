#include "expoparams.hpp"
#include <npm/common/util.hpp>

using namespace npm;
using namespace boost;

expoparams::
expoparams(shared_ptr<sfl::OptionDictionary> opt)
{
  expo_default_parameters(this);
  model_axlewidth = 0.8;
  model_phi_max = 1.0;
  model_phid_max = 3.0;
  if( ! opt)
    return;
  
  string_to(opt->GetOption("front_channel"), front_channel);
  string_to(opt->GetOption("front_nscans"), front_nscans);
  string_to(opt->GetOption("front_mount_x"), front_mount_x);
  string_to(opt->GetOption("front_mount_y"), front_mount_y);
  string_to(opt->GetOption("front_mount_theta"), front_mount_theta);
  string_to(opt->GetOption("front_rhomax"), front_rhomax);
  string_to(opt->GetOption("front_phi0"), front_phi0);
  string_to(opt->GetOption("front_phirange"), front_phirange);
  
  string_to(opt->GetOption("rear_channel"), rear_channel);
  string_to(opt->GetOption("rear_nscans"), rear_nscans);
  string_to(opt->GetOption("rear_mount_x"), rear_mount_x);
  string_to(opt->GetOption("rear_mount_y"), rear_mount_y);
  string_to(opt->GetOption("rear_mount_theta"), rear_mount_theta);
  string_to(opt->GetOption("rear_rhomax"), rear_rhomax);
  string_to(opt->GetOption("rear_phi0"), rear_phi0);
  string_to(opt->GetOption("rear_phirange"), rear_phirange);
  
  string_to(opt->GetOption("model_security_distance"),
	    model_security_distance);
  string_to(opt->GetOption("model_wheelbase"), model_wheelbase);
  string_to(opt->GetOption("model_wheelradius"), model_wheelradius);
  string_to(opt->GetOption("model_axlewidth"), model_axlewidth);
  string_to(opt->GetOption("model_qd_max"), model_qd_max);
  string_to(opt->GetOption("model_qdd_max"), model_qdd_max);
  string_to(opt->GetOption("model_sd_max"), model_sd_max);
  string_to(opt->GetOption("model_thetad_max"), model_thetad_max);
  string_to(opt->GetOption("model_sdd_max"), model_sdd_max);
  string_to(opt->GetOption("model_thetadd_max"), model_thetadd_max);
  string_to(opt->GetOption("model_phi_max"), model_phi_max);
  string_to(opt->GetOption("model_phid_max"), model_phid_max);
  
  string_to(opt->GetOption("dwa_dimension"), dwa_dimension);
  string_to(opt->GetOption("dwa_grid_width"), dwa_grid_width);
  string_to(opt->GetOption("dwa_grid_height"), dwa_grid_height);
  string_to(opt->GetOption("dwa_grid_resolution"), dwa_grid_resolution);
  string_to(opt->GetOption("dwa_alpha_distance"), dwa_alpha_distance);
  string_to(opt->GetOption("dwa_alpha_heading"), dwa_alpha_heading);
  string_to(opt->GetOption("dwa_alpha_speed"), dwa_alpha_speed);
  
  string_to_bool(opt->GetOption("bband_enabled"), bband_enabled);
  string_to(opt->GetOption("bband_shortpath"), bband_shortpath);
  string_to(opt->GetOption("bband_longpath"), bband_longpath);
  string_to(opt->GetOption("bband_maxignoredistance"),
	    bband_maxignoredistance);
  
  string_to_bool(opt->GetOption("mp_strict_dwa"), mp_strict_dwa);
  string_to_bool(opt->GetOption("mp_auto_adapt_dwa"),
		 mp_auto_adapt_dwa);
  string_to(opt->GetOption("mp_dtheta_starthoming"),
	    mp_dtheta_starthoming);
  string_to(opt->GetOption("mp_dtheta_startaiming"),
	    mp_dtheta_startaiming);
}
