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


#ifndef NPM_SIMULATOR_HPP
#define NPM_SIMULATOR_HPP


#include <sfl/util/Pthread.hpp>
#include <boost/shared_ptr.hpp>
#include <string>
#include <vector>


namespace npm {


  class World;
  class RobotClient;


  class Simulator
  {
  public:
    Simulator(boost::shared_ptr<npm::World> world, double timestep,
	      boost::shared_ptr<sfl::Mutex> mutex);
    ~Simulator();
  
    void InitRobots(const std::string & filename);
    void InitLayout(const std::string & filename, bool fatal_warnings);
    void Init();
    bool Idle();
    void Reshape(int width, int height);
    void Draw();
    void Keyboard(unsigned char key, int x, int y);
  
    void SetContinuous(bool printscreen = false);
  
  private:
  
    struct robot_s {
      robot_s(boost::shared_ptr<npm::RobotClient> _robot)
	: robot(_robot), runnable(true) {}
      boost::shared_ptr<npm::RobotClient> robot;
      bool runnable;
    };
  
    typedef std::vector<robot_s> robot_t;
  
    void PrintScreen();
    void UpdateAllSensors();
    void UpdateRobots();
  
    boost::shared_ptr<npm::World> m_world;  
    robot_t m_robot;
    bool m_step;
    bool m_continuous;
    bool m_printscreen;
    int m_width, m_height;
    double m_timestep;
    boost::shared_ptr<sfl::Mutex> m_mutex;  
  };

}

#endif // NPM_SIMULATOR_HPP
