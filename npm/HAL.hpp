/* 
 * Copyright (C) 2005
 * Swiss Federal Institute of Technology, Lausanne. All rights reserved.
 * 
 * Author: Roland Philippsen <roland dot philippsen at gmx dot net>
 *         Autonomous Systems Lab <http://asl.epfl.ch/>
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


#ifndef NPM_HAL_HPP
#define NPM_HAL_HPP


#include <sfl/api/HAL.hpp>


namespace npm {
  
  
  class RobotServer;
  class NoiseModel;
  
  
  class HAL
    : public sfl::HAL
  {
  public:
    explicit HAL(RobotServer * owner);
    virtual ~HAL();
    
    virtual int speed_set(const double * qdot, size_t * qdot_len);
    virtual int speed_get(double * qdot, size_t * qdot_len);
    
  protected:
    friend class RobotServer;
    
    virtual void UpdateSpeeds();
    void EnableOdometryNoise(const NoiseModel * noise);
    void DisableOdometryNoise();
    void EnableScannerNoise();
    void DisableScannerNoise();
    
  private:
    size_t m_ndof;
    RobotServer * m_owner;
    double *m_wanted_speed;
    double *m_current_speed;
    const NoiseModel * m_odometry_noise;
    bool m_noisy_scanners;
  };
  
}

#endif // NPM_HAL_HPP
