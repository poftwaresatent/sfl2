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


#ifndef ESTAR_DRAWING_HPP
#define ESTAR_DRAWING_HPP


#include <npm/common/Drawing.hpp>
#include <boost/shared_ptr.hpp>


namespace estar {
  class Facade;
}


namespace sfl {
  class GridFrame;
}


class PlanProxy {
public:
  virtual const estar::Facade * GetFacade() = 0;
  virtual const sfl::GridFrame * GetFrame() = 0;
};


class EstarDrawing
  : public npm::Drawing
{
public:
  typedef enum { VALUE, META, QUEUE, UPWIND } what_t;
  
  what_t what;
  
  EstarDrawing(const std::string & name,
	       boost::shared_ptr<PlanProxy> proxy,
	       what_t what);
  
  virtual void Draw();
  
private:
  boost::shared_ptr<PlanProxy> m_proxy;
};

#endif // ESTAR_DRAWING_HPP
