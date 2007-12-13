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


#include "World.hpp"
#include "Object.hpp"
#include "RobotServer.hpp"
#include "WorldDrawing.hpp"
#include "WorldCamera.hpp"
#include "BBox.hpp"
#include "Random.hpp"
#include <sfl/util/pdebug.hpp>
#include <sfl/util/Line.hpp>
#include <sfl/gplan/TraversabilityMap.hpp>
#include <iostream>
#include <sstream>
#include <cmath>


#ifdef NPM_DEBUG
# define PDEBUG PDEBUG_OUT
#else // ! NPM_DEBUG
# define PDEBUG PDEBUG_OFF
#endif // NPM_DEBUG
#define PVDEBUG PDEBUG_OFF


using namespace sfl;
using namespace boost;
using namespace std;


namespace npm {
  
  
  World::
  World(const string & name)
    : m_drawing(new WorldDrawing(name, * this)),
      m_camera(new WorldCamera(name, * this))
  {
    m_object.push_back(shared_ptr<Object>(new Object(name, "THE world")));
  }
  
  
  shared_ptr<World> World::
  Create(const string & name)
  {
    if(name == "mini")
      return Mini();
    if(name == "tta")
      return TicketToAcapulco();
    if(name == "expo")
      return Expo();
    if(name == "stage")
      return Stage();
    return shared_ptr<World>();
  }
  
  
  shared_ptr<World> World::
  Mini()
  {
    shared_ptr<World> mini(new World("mini"));
    
    mini->AddLine(Line(0, 3, 1, 3));
    mini->AddLine(Line(-1, -1, 4, -1));
    mini->AddLine(Line(4, -1, 4, 4));
    mini->AddLine(Line(4, 4, -1, 4));
    mini->AddLine(Line(-1, 4, -1, -1));
    
    return mini;
  }


  shared_ptr<World> World::
  Expo()
  {
    shared_ptr<World> expo(new World("expo"));

    expo->AddLine(Line( 0  ,  8  , 12  ,  3  ));
    expo->AddLine(Line(12  ,  3  , 24  ,  2.3));
    expo->AddLine(Line(24  ,  2.3, 18  , 24  ));
    expo->AddLine(Line(18  , 24  , 18  , 28.9));
    expo->AddLine(Line( 9.4, 27.7, 15.1, 28.9));
    expo->AddLine(Line(15.1, 28.9, 14  , 24  ));
    expo->AddLine(Line(14  , 24  , 10  , 24  ));
    expo->AddLine(Line(10  , 24  ,  4.6, 21.3));
    expo->AddLine(Line( 4.6, 21.3,  2.6, 22.4));
    expo->AddLine(Line( 2.6, 22.4,  1.4, 18  ));
    expo->AddLine(Line( 1.4, 18  ,  3.6, 18  ));
    expo->AddLine(Line( 3.6, 18  ,  6  , 12  ));
    expo->AddLine(Line( 6  , 12  , 11.4,  9.9));
    expo->AddLine(Line(11.4,  9.9, 11  ,  8.6));
    expo->AddLine(Line(11  ,  8.6,  5.1, 10.3));

    Object column("column", "");
    column.AddLine(Line(-0.2, -0.2,  0.2, -0.2));
    column.AddLine(Line( 0.2, -0.2,  0.2,  0.2));
    column.AddLine(Line( 0.2,  0.2, -0.2,  0.2));
    column.AddLine(Line(-0.2,  0.2, -0.2, -0.2));
    for(int x(6); x <= 18; x += 6)
      for(int y(6); y <= 18; y += 6){
	Object col(column);
	col.TransformTo(Frame(x, y, 0));
	expo->AddObject(col);
      }

    Object biotop("biotop", "");
    biotop.AddLine(Line(-1.3, 0, 0, -1.3));
    biotop.AddLine(Line(0, -1.3, 3.6, 0));
    biotop.AddLine(Line(3.6, 0, 0, 1.3));
    biotop.AddLine(Line(0, 1.3, -1.3, 0));
    biotop.TransformTo(Frame(12, 16.3, 7 * M_PI / 180));
    expo->AddObject(biotop);

    return expo;
  }


  shared_ptr<World> World::
  TicketToAcapulco()
  {
    shared_ptr<World> acapulco(new World("acapulco"));

    //////////////////////////////////////////////////
    // lines
    acapulco->AddLine(Line(-2,  -2, 12.01, -2)); // hax .01 for visibility
    acapulco->AddLine(Line(12,  -2, 12,  8));
    acapulco->AddLine(Line(12,   8, -2,  8));
    acapulco->AddLine(Line(-2,   8, -2, -2));
    acapulco->AddLine(Line( 0, 6.5,  2,  4));
    acapulco->AddLine(Line( 2,   4,  7,  4));


    //////////////////////////////////////////////////
    // objects
    //   class Object *column = new Object("column");
    //   column->SetPose(6, 6, 0);
    //   column->AddLine(-0.2, -0.2,  0.2, -0.2);
    //   column->AddLine( 0.2, -0.2,  0.2,  0.2);
    //   column->AddLine( 0.2,  0.2, -0.2,  0.2);
    //   column->AddLine(-0.2,  0.2, -0.2, -0.2);
    //   acapulco->AddObject(column, true);
    //   acapulco->AddObject(column->Clone(12, 6, 0), true);

    //////////////////////////////////////////////////
    // visitors
#ifdef UNDEFINED
    double x[] = {2.5, 4.5, 6  ,  9, 9.5, 11};
    double y[] = {3  , 1  , 3.5, -1, 1.5,  3};
    for(int i = 0; i < 6; ++i){
      Visitor *visitor = new Visitor();
      visitor->Create(acapulco);
      visitor->SetPose(x[i], y[i], 0);
      visitor->AddPose(x[i] + 3, y[i] + 3, 0);
      acapulco->AddMovingObject(visitor, true);
    }
#endif
  
    return acapulco;
  }
  
  
  void World::
  AddLine(const Line & line)
  {
    m_object[0]->AddLine(line);
    if( ! m_bbox)
      m_bbox.reset(new BBox(line));
    else
      m_bbox->Update(line);
  }
  
  
  void World::
  AddObject(const Object & object)
  {
    m_object.push_back(shared_ptr<Object>(new Object(object)));
    if( ! m_bbox)
      m_bbox.reset(new BBox(*object.GetGlobalBB()));
    else
      m_bbox->Update(*object.GetGlobalBB());
  }
  
  
  void World::
  UpdateSensor(Sensor & sensor) const
  {
    for(size_t io(0); io < m_object.size(); ++io)
      m_object[io]->UpdateSensor(sensor);
    for(size_t ir(0); ir < m_robot.size(); ++ir)
      m_robot[ir]->UpdateSensor(sensor);
  }
  
  
  void World::
  AddRobot(RobotServer * robot)
  {
    m_robot.push_back(robot);
  }
  
  
  shared_ptr<World> World::
  Stage()
  {
    shared_ptr<World> stage(new World("stage"));
    
    //////////////////////////////////////////////////
    // H columns
    {
      Object column("H column", "");
      
      static const double width(0.303);
      static const double height(0.15);
      static const double thick(0.02);
      static const double empty(0.065);
    
      // bottom hollow
      column.AddLine(Line(0,             0,      thick,         0));
      column.AddLine(Line(thick,         0,      thick,         empty));
      column.AddLine(Line(thick,         empty,  width - thick, empty));
      column.AddLine(Line(width - thick, empty,  width - thick, 0));
      column.AddLine(Line(width - thick, 0,      width,         0));

      // top hollow
      column.AddLine(Line(0,           height,       thick,     height));
      column.AddLine(Line(thick,       height,       thick,     height-empty));
      column.AddLine(Line(thick,     height-empty, width-thick, height-empty));
      column.AddLine(Line(width-thick, height-empty, width-thick, height));
      column.AddLine(Line(width-thick, height,        width,      height));

      // sides
      column.AddLine(Line(0,     0,     0, height));
      column.AddLine(Line(width, 0, width, height));

      Frame pos[14];
    
      pos[ 0] = Frame(0.0000, 0.0000, 0);
      pos[ 1] = Frame(0.0000, 1.9950, 0);
      pos[ 2] = Frame(0.0000, 4.0650, 0);
      pos[ 3] = Frame(0.0000, 6.1350, 0);
    
      pos[ 4] = Frame(8.5730, 0.0000, 0);
      pos[ 5] = Frame(8.5730, 1.9950, 0);
      pos[ 6] = Frame(8.5730, 4.0650, 0);
      pos[ 7] = Frame(8.5730, 6.1350, 0);
    
      pos[ 8] = Frame(2.1085, 7.6370, M_PI / 2);
      pos[ 9] = Frame(6.8425, 7.6370, M_PI / 2);
      pos[10] = Frame(3.9405, 7.6370, M_PI / 2);
      pos[11] = Frame(5.0855, 7.6370, M_PI / 2);
      pos[12] = Frame(3.9405, 6.1370, M_PI / 2);
      pos[13] = Frame(5.0855, 6.1370, M_PI / 2);
    
      for(int i(0); i < 14; ++i){
	Object col(column);
	col.TransformTo(pos[i]);
	stage->AddObject(col);
      }
    }
  
    //////////////////////////////////////////////////
    // round columns
    {
      Object column("round column", "");
    
      static const double radius(0.04);
      static const int nlines(16);
      static const double dtheta(2 * M_PI / nlines);
      for(int i(0); i < nlines; ++i)
	column.AddLine(Line(radius * cos(i * dtheta),
			    radius * sin(i * dtheta),
			    radius * cos(i * dtheta + dtheta),
			    radius * sin(i * dtheta + dtheta)));
    
      Frame pos[7];
    
      pos[ 0] = Frame(1.4970, 6.2100, 0);
      pos[ 1] = Frame(6.1320, 6.2100, 0);
      pos[ 2] = Frame(7.3790, 6.2100, 0);

      pos[ 3] = Frame(2.7440, 4.9210, 0);
      pos[ 4] = Frame(6.1320, 4.9210, 0);

      pos[ 5] = Frame(2.0330, 4.2090, 0);
      pos[ 6] = Frame(6.8430, 4.2090, 0);
    
      for(int i(0); i < 7; ++i){
	Object col(column);
	col.TransformTo(pos[i]);
	stage->AddObject(col);
      }
    }

    //////////////////////////////////////////////////  
    // square columns
    {
      Object column("square column", "");
    
      static const double width(0.25);
      column.AddLine(Line(    0,     0, width,     0));
      column.AddLine(Line(width,     0, width, width));
      column.AddLine(Line(width, width,     0, width));
      column.AddLine(Line(    0, width,     0,     0));

      Frame pos[2];
      pos[ 0] = Frame(0.0000, 7.6900, 0);
      pos[ 1] = Frame(8.6260, 7.6900, 0);
    
      for(int i(0); i < 2; ++i){
	Object col(column);
	col.TransformTo(pos[i]);
	stage->AddObject(col);
      }
    }
  
    //////////////////////////////////////////////////
    // walls
  
    stage->AddLine(Line(0.0000, 0.0000, 0.0000, 7.9400));
    stage->AddLine(Line(8.8760, 0.0000, 8.8760, 7.9400));
    stage->AddLine(Line(0.0000, 7.9400, 8.8760, 7.9400));
  
    return stage;
  }


  void World::
  DumpLines(ostream & os, bool use_windows_eol) const
  {
    string eol;
    if(use_windows_eol)
      eol = "\r\n";
    else
      eol = "\n";
  
    os << "# map of \"" << m_object[0]->name
       << "\": floor x0 y0 x1 y1" << eol
       << "1" << eol
       << "1\tfloor" << eol;
    for(size_t io(0); io < m_object.size(); ++io)
      for(size_t il(0); il < m_object[io]->GetNlines(); ++il){
	shared_ptr<const Line> foo(m_object[io]->GetGlobalLine(il));
	os << "1\t" << foo->X0() << "\t" << foo->Y0()
	   << "\t" << foo->X1() << "\t" << foo->Y1() << eol;
      }
  }
  
  
  void World::
  ApplyTraversability(const TraversabilityMap & travmap, bool simple)
  {
    GridFrame const & gframe(travmap.gframe);
    double const offset(gframe.Delta() / 2);
    int const obstacle(travmap.obstacle);
    ssize_t const xbegin(travmap.grid.xbegin());
    ssize_t const xend(travmap.grid.xend());
    ssize_t const ybegin(travmap.grid.ybegin());
    ssize_t const yend(travmap.grid.yend());
    
    for (ssize_t ix(xbegin); ix < xend; ++ix)
      for (ssize_t iy(ybegin); iy < yend; ++iy)
	if (travmap.grid.at(ix, iy) >= obstacle) {
	  GridFrame::position_t center(gframe.LocalPoint(ix, iy));
	  if (simple) {
	    {
	      Line line(center.v0 - offset, center.v1,
			center.v0 + offset, center.v1);
	      line.TransformTo(gframe);
	      AddLine(line);
	    }
	    {
	      Line line(center.v0, center.v1 - offset,
			center.v0, center.v1 + offset);
	      line.TransformTo(gframe);
	      AddLine(line);
	    }
	  }
	  else {
	    try { // west
	      if (travmap.grid.at(ix - 1, iy) < obstacle) {
		Line line(center.v0 - offset, center.v1 - offset,
			  center.v0 - offset, center.v1 + offset);
		line.TransformTo(gframe);
		AddLine(line);
	      }
	    }
	    catch (out_of_range) {
	    }
	    try { // east
	      if (travmap.grid.at(ix + 1, iy) < obstacle) {
		Line line(center.v0 + offset, center.v1 - offset,
			  center.v0 + offset, center.v1 + offset);
		line.TransformTo(gframe);
		AddLine(line);
	      }
	    }
	    catch (out_of_range) {
	    }
	    try { // south
	      if (travmap.grid.at(ix, iy - 1) < obstacle) {
		Line line(center.v0 - offset, center.v1 - offset,
			  center.v0 + offset, center.v1 - offset);
		line.TransformTo(gframe);
		AddLine(line);
	      }
	    }
	    catch (out_of_range) {
	    }
	    try { // north
	      if (travmap.grid.at(ix, iy + 1) < obstacle) {
		Line line(center.v0 - offset, center.v1 + offset,
			  center.v0 + offset, center.v1 + offset);
		line.TransformTo(gframe);
		AddLine(line);
	      }
	    }
	    catch (out_of_range) {
	    }
	  } // if (simple) ... else
	}
    
    // lower left
    GridFrame::position_t corner(gframe.LocalPoint(xbegin, ybegin));
    corner.v0 -= offset;
    corner.v1 -= offset;
    gframe.To(corner.v0, corner.v1);
    if( ! m_bbox)
      m_bbox.reset(new BBox(corner.v0, corner.v1, corner.v0, corner.v1));
    else
      m_bbox->Update(corner.v0, corner.v1);
    
    // upper left
    corner = gframe.LocalPoint(xbegin, yend - 1);
    corner.v0 -= offset;
    corner.v1 += offset;
    gframe.To(corner.v0, corner.v1);
    m_bbox->Update(corner.v0, corner.v1);
    
    // lower right
    corner = gframe.LocalPoint(xend - 1, ybegin);
    corner.v0 += offset;
    corner.v1 -= offset;
    gframe.To(corner.v0, corner.v1);
    m_bbox->Update(corner.v0, corner.v1);
    
    // upper right
    corner = gframe.LocalPoint(xend - 1, yend - 1);
    corner.v0 += offset;
    corner.v1 += offset;
    gframe.To(corner.v0, corner.v1);
    m_bbox->Update(corner.v0, corner.v1);
  }
  
  
  shared_ptr<World> World::
  Parse(istream & is, ostream * os)
  {
    string name("");
    vector<Line> line;
    string textline;
    double probability(-1);
    while(getline(is, textline)){
      istringstream tls(textline);
      if(textline[0] == '#')
	continue;
      string token;
      if( ! (tls >> token))
	continue;
      if(token == "name"){
	tls >> name;
	if( ! tls){
	  if(os) *os << "ERROR: could not parse name from \""
		     << tls.str() << "\"\n";
	  return shared_ptr<World>();
	}
	PVDEBUG("name: %s\n", name.c_str());
      }
      else if(token == "line"){
	double x0, y0, x1, y1;
	tls >> x0 >> y0 >> x1 >> y1;
	if( ! tls){
	  if(os) *os << "ERROR: could not parse line from \""
		     << tls.str() << "\"\n";
	  return shared_ptr<World>();
	}
	line.push_back(Line(x0, y0, x1, y1));
	PVDEBUG("line %g  %g  %g  %g\n", x0, y0, x1, y1);
      }
      else if(token == "probability"){
	tls >> probability;
	if( ! tls){
	  if(os) *os << "ERROR: could not parse probability from \""
		     << tls.str() << "\"\n";
	  return shared_ptr<World>();
	}
	PVDEBUG("probability %g\n", probability);
      }
      else{
	if(os) *os << "ERROR: could not parse \""
		   << tls.str() << "\"\n";
	return shared_ptr<World>();
      }
    }
    if(name.empty()){
      if(os) *os << "ERROR: no name specified\n";
      return shared_ptr<World>();
    }
    if(line.empty()){
      if(os) *os << "ERROR: no lines specified\n";
      return shared_ptr<World>();
    }
    shared_ptr<World> world(new World(name));
    if(probability < 0)
      for(size_t il(0); il < line.size(); ++il)
	world->AddLine(line[il]);
    else
      for(size_t il(0); il < line.size(); ++il)
	if(Random::Uniform(probability))
	  world->AddLine(line[il]);
    PVDEBUG("bbox %g  %g  %g  %g\n", world->m_bbox->X0(), world->m_bbox->Y0(),
	    world->m_bbox->X1(), world->m_bbox->Y1());
    return world;
  }
  
  
  void World::
  AddKeyListener(boost::shared_ptr<KeyListener> listener) const
  {
    m_listener.push_back(listener);
  }
  
  
  void World::
  DispatchKey(unsigned char key) const
  {
    for(size_t il(0); il < m_listener.size(); ++il)
      m_listener[il]->KeyPressed(key);
  }
  
}

