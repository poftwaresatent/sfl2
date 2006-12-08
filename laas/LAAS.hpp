/* 
 * Copyright (C) 2006 Roland Philippsen <roland dot philippsen at gmx dot net>
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


#ifndef LAAS_HPP
#define LAAS_HPP


#include <npm/common/RobotClient.hpp>
#include <boost/scoped_ptr.hpp>


namespace sfl {
  class RobotModel;
  class Hull;
}


class GenomBridge;


class LAAS
  : public npm::RobotClient
{
private:
  LAAS(const LAAS &);
  
protected:
  LAAS(boost::shared_ptr<sfl::Hull> hull,
       boost::shared_ptr<npm::RobotDescriptor> descriptor,
       const npm::World & world);
  
public:
  ~LAAS();
  
  virtual void PrepareAction(double timestep);
  virtual void InitPose(double x, double y, double theta);
  virtual void SetPose(double x, double y, double theta);
  virtual void GetPose(double & x, double & y, double & theta);
  virtual boost::shared_ptr<const sfl::Goal> GetGoal();
  virtual bool GoalReached();
  virtual void SetGoal(double timestep, const sfl::Goal & goal);
  
protected:
  friend class GenomHAL;
  friend class GenomHALFactory;
  
  void CreateGfxStuff(const std::string & name);
  
  boost::shared_ptr<npm::Lidar> m_front;
  boost::shared_ptr<npm::DiffDrive> m_drive;
  boost::shared_ptr<sfl::RobotModel> m_robotModel;
  boost::shared_ptr<sfl::Hull> m_hull;
  boost::shared_ptr<sfl::Goal> m_goal;
  boost::shared_ptr<GenomBridge> m_bridge;
};


class Jido:
  public LAAS
{
public:
  Jido(boost::shared_ptr<npm::RobotDescriptor> descriptor,
       const npm::World & world);
};


class Rackham:
  public LAAS
{
public:
  Rackham(boost::shared_ptr<npm::RobotDescriptor> descriptor,
	  const npm::World & world);
};

#endif // LAAS_HPP
