/* 
 * Copyright (C) 2004
 * Swiss Federal Institute of Technology, Lausanne. All rights reserved.
 * 
 * Developed at the Autonomous Systems Lab.
 * Visit our homepage at http://asl.epfl.ch/
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


#ifndef NPM_LIDAR_HPP
#define NPM_LIDAR_HPP


#include <npm/Sensor.hpp>
#include <vector>


namespace sfl {
  class Scanner;
}


namespace npm {
  
  class ScannerDrawing;
  class HAL;
  class NoiseModel;
  
  /**
     Simulates a range scanner.
  */
  class Lidar
    : public Sensor
  {
  private:
    friend class RobotServer;
    
    /** The constructor is private such that only friends can create
	lidars. This is accomplished through
	RobotServer::DefineLidar().
    */
    Lidar(const RobotServer * owner, boost::shared_ptr<HAL> hal,
	  const sfl::Frame & mount,
	  size_t nscans, double rhomax,
	  boost::shared_ptr<sfl::Scanner> scanner,
	  boost::shared_ptr<NoiseModel> noise_model);
    
    /** non-copyable */
    Lidar(const Lidar &);
    
  public:
    virtual void InitUpdate();
    virtual void StepUpdate(const sfl::Line & line);
    virtual void FinalizeUpdate();
    
    boost::shared_ptr<sfl::Scanner> GetScanner() { return m_scanner; }
    boost::shared_ptr<const sfl::Scanner> GetScanner() const
    { return m_scanner; }
    
    /** \pre index < nscans */
    double GetTrueRho(size_t index) const { return m_true_rho[index]; }
    /** \pre index < nscans */
    double GetNoisyRho(size_t index) const { return m_noisy_rho[index]; }
    struct timespec GetT0() const { return m_t0; }
    struct timespec GetT1() const { return m_t1; }
    const sfl::Frame & GetGlobalPose() const { return * m_global_pose; }
    
    bool HaveNoiseModel() const { return m_noise_model.get() != 0; }
    
    const size_t nscans;
    const double rhomax;
    const boost::shared_ptr<const sfl::Frame> mount;
    
  private:
    boost::shared_ptr<HAL> m_hal;
    boost::shared_ptr<sfl::Scanner> m_scanner;
    boost::shared_ptr<NoiseModel> m_noise_model;
    boost::shared_ptr<sfl::Frame> m_global_pose;
    boost::shared_ptr<ScannerDrawing> m_drawing;
    std::vector<double> m_true_rho;
    std::vector<double> m_noisy_rho;
    struct timespec m_t0, m_t1;
  };
  
}

#endif // NPM_LIDAR_HPP
