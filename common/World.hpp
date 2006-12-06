/* 
 * Copyright (C) 2004
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


#ifndef NPM_WORLD_HPP
#define NPM_WORLD_HPP


#include <boost/shared_ptr.hpp>
#include <vector>


namespace sfl {
  class Line;
  class TraversabilityMap;
}


namespace npm {

  class Object;  
  class RobotServer;  
  class WorldDrawing;
  class WorldCamera;
  class BBox;
  class Sensor;
  
  /**
     Simulated world. Basically a container for objects and robots.
     
     \todo how about storing robots here with smart pointers?
  */
  class World
  {
  public:
    /** \note Beware of boost::shared_ptr and the fact that Robot
	instances have shared pointers to sensors which might have
	shared pointers to the world... CAREFUL! */
    typedef std::vector<boost::shared_ptr<Object> > object_t;
    
    World(const std::string & name);
    
    /** Factory method for runtime configuration */
    static boost::shared_ptr<World> Create(const std::string & name);
    static boost::shared_ptr<World> TicketToAcapulco();
    static boost::shared_ptr<World> Mini();
    static boost::shared_ptr<World> Expo();
    static boost::shared_ptr<World> Stage();
    
    
    /**
       Adds lines to the world, based on the boundary between obstcale
	and non-obstacle cells in the given traversability map. Cells
	with a value of travmap.obstacle or higher are considered
	obstacles.
	
	\todo Maybe add a way for traversability values to be
	interpreted according to an inverted scale.
    */
    void
    ApplyTraversability(boost::shared_ptr<sfl::TraversabilityMap> travmap);
    
    void AddLine(const sfl::Line & line);
    
    /** Makes a copy of the argument. */
    void AddObject(const Object & object);
    
    void UpdateSensor(Sensor & sensor) const;
    
    /** \note Unlike World::AddObject(), which copy constructs the
	object and deletes it in World::~World(), the robots are kept in
	a list for reference only (Sensor updates) and are not
	deleted. */
    void AddRobot(RobotServer * robot);
    
    /** \note the world's "own" lines are stored in the first element */
    const object_t & GetObjects() const { return m_object; }
    
    /** \note Can return 0. */
    boost::shared_ptr<const BBox> GetBounds() const { return m_bbox; }
    
    /** For dumping the map in a textfile. */
    void DumpLines(std::ostream & os, bool use_windows_eol) const;
    
  private:
    friend class CheatSheet;
    
    typedef std::vector<RobotServer *> robot_t;
    
    robot_t m_robot;
    boost::shared_ptr<WorldDrawing> m_drawing;
    boost::shared_ptr<WorldCamera> m_camera;
    object_t m_object;
    boost::shared_ptr<BBox> m_bbox;
    boost::shared_ptr<sfl::TraversabilityMap> m_travmap;
  };
  
}

#endif // NPM_WORLD_HPP
