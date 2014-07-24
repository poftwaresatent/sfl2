# Pose of the robot when this scan was taken (px, py, pth)
#
float64 px
float64 py
float64 pth

# Laser mount (x, y, theta) with respect to the robot frame
#
float64 mx
float64 my
float64 mth

# Laser configuration
#  nscans: number of rays in a scan
#  rhomax: maximum distance [m]
#  phi0: angle [rad] of first ray
#  phirane: angle range between first and last ray [rad]
#
int32   nscans
float64 rhomax
float64 phi0
float64 phirange

# Raw laser data [m]
#
float64[] rho
