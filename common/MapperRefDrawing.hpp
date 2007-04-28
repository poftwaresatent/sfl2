/* -*- mode: C++; tab-width: 2 -*- */
/* 
 * Copyright (C) 2007
 * Swiss Federal Institute of Technology, Zurich. All rights reserved.
 * 
 * Developed at the Autonomous Systems Lab.
 * Visit our homepage at http://www.asl.ethz.ch/
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


#ifndef NPM_MAPPER_REF_DRAWING_HPP
#define NPM_MAPPER_REF_DRAWING_HPP


#include <npm/common/Drawing.hpp>


namespace sfl {
  class Mapper2d;
}


namespace npm {
  
  
  class MapperRefDrawing
    : public Drawing
  {
  public:
    MapperRefDrawing(const std::string & name,
										 boost::shared_ptr<const sfl::Mapper2d> mapper,
										 bool draw_link);
    
    virtual void Draw();
    		
  private:
    boost::shared_ptr<const sfl::Mapper2d> m_mapper;
		bool m_draw_link;					 // either draw the link data, or ref data
  };
	
}

#endif // NPM_MAPPER_REF_DRAWING_HPP
