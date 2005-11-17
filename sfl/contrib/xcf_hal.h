#ifndef XCF_HAL_H
#define XCF_HAL_H

#ifdef __cplusplus
extern "C" {
#endif // __cplusplus
  
  int xcf_odometry_init();
  
  int xcf_odometry_get(double x_mm, double y_mm, double theta_rad);

  int xcf_odometry_end();
  


  int xcf_speed_init();
  
  int xcf_speed_set(double v, double w);

  int xcf_speed_end();


  
  int xcf_scan_init(int channel);
  
  int xcf_scan_get(int channel, double * rho, int rho_len,
		   uint64_t * timestamp_ms);
  
  int xcf_scan_end(int channel);
  
#ifdef __cplusplus
}
#endif // __cplusplus
  
#endif // XCF_HAL_H
