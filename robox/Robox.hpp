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


#ifndef ROBOX_HPP
#define ROBOX_HPP


#include <npm/common/RobotClient.hpp>


//rfct
#include <sfl/expo/Robox.hpp>
#include <vector>
// namespace expo {
//   class Robox;
// }


namespace local {
  class NGKeyListener;
}


namespace npm {
  
  class VisualRobox
    : public expo::Robox
  {
  public:
    VisualRobox(std::string const & name,
		expo_parameters const & params,
		boost::shared_ptr<sfl::HAL> hal,
		boost::shared_ptr<sfl::Multiscanner> mscan);
    
  protected:
    void AddDrawing(Drawing * drawing);
    void AddCamera(Camera * camera);
    
    std::vector<boost::shared_ptr<Drawing> > m_drawing;
    std::vector<boost::shared_ptr<Camera> > m_camera;
  };
  
}


class Robox
  : public npm::RobotClient
{
private:
  Robox(const Robox &);
  
protected:
  Robox(boost::shared_ptr<npm::RobotDescriptor> descriptor,
	const npm::World & world);
  
public:
  static Robox *
  Create(boost::shared_ptr<npm::RobotDescriptor> descriptor,
	 const npm::World & world);
  
  virtual bool PrepareAction(double timestep);
  virtual void SetGoal(double timestep, const sfl::Goal & goal);
  virtual bool GoalReached();
  virtual void InitPose(double x, double y, double theta);
  virtual void SetPose(double x, double y, double theta);
  virtual void GetPose(double & x, double & y, double & theta);
  virtual boost::shared_ptr<const sfl::Goal> GetGoal();
  
protected:
  boost::shared_ptr<expo::Robox> m_base;
  boost::shared_ptr<npm::DiffDrive> m_drive;
  boost::shared_ptr<local::NGKeyListener> m_ngkl;
  
  bool StartThreads();
};

#endif // ROBOX_HPP
