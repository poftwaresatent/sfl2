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


#include <npm/common/Manager.hpp>
#include <npm/common/View.hpp>
#include <boost/shared_ptr.hpp>
#include <string>
#include <vector>


namespace npm {


  class World;
  class RobotClient;
  class Simulator;
  
  
  class AppWindow
  {
  public:
    std::string const name;
    std::string const layout_filename;
    
    AppWindow(std::string const & name, std::string const & layout_filename,
	      int width, int height, Simulator * simul);
    
    void InitLayout();
    void Reshape(int width, int height);
    void Draw();
    void Keyboard(unsigned char key, int x, int y);
    void PrintScreen();
    
    void GetSize(int & width, int & height) { width = m_width; height = m_height; }
    
  private:
    Simulator * m_simul;
    
    int m_width, m_height;
    
    typedef SubManager<View> layout_t;
    typedef boost::shared_ptr<layout_t> layout_ptr;
    typedef std::map<unsigned char, layout_ptr> layout_map_t;
    
    layout_map_t m_layout;
    layout_ptr m_active_layout;
    layout_ptr m_default_layout;
    
    typedef boost::shared_ptr<View> view_ptr;
    typedef std::vector<view_ptr> views_t;
    
    views_t m_views;
  };
  
  
  class Simulator
  {
  public:
    std::string const robot_config_filename; // rfct
    bool const fatal_warnings;	// rfct
    
    Simulator(boost::shared_ptr<World> world, double timestep,
	      std::string const & robot_config_filename,
	      std::string const & layout_filename,
	      bool fatal_warnings);
    ~Simulator();
  
    void InitRobots();
    void Init();
    bool Idle();
  
    void SetContinuous(bool printscreen = false);
    
    size_t GetNAppWindows() const { return m_appwin.size(); } // rfct?
    AppWindow * GetAppWindow(size_t ii) { return m_appwin[ii].get(); } // rfct?
    
  private:
    friend class AppWindow;	// rfct
    std::vector<boost::shared_ptr<AppWindow> > m_appwin;
    
    struct robot_s {
      robot_s(boost::shared_ptr<RobotClient> _robot)
	: robot(_robot), runnable(true) {}
      boost::shared_ptr<RobotClient> robot;
      bool runnable;
    };
  
    typedef std::vector<robot_s> robot_t;
  
    void UpdateAllSensors();
    void UpdateRobots();
  
    boost::shared_ptr<World> m_world;  
    robot_t m_robot;
    bool m_step;
    bool m_continuous;
    bool m_printscreen;
    double m_timestep;
  };

}

#endif // NPM_SIMULATOR_HPP
