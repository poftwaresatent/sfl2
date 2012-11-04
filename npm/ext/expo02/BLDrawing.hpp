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


#ifndef BLDRAWING_H
#define BLDRAWING_H


#include <npm/gfx/Drawing.hpp>
#include <sfl/bband/BubbleList.hpp>
#include <string>


class BLDrawing
  : public npm::Drawing
{
public:
  typedef enum { FULL, REDUCED, SNAPPED } mode_t;

  BLDrawing(const std::string & name,
	    const sfl::BubbleList * bubble_list,
	    mode_t mode = FULL,
	    double intensity = 1);

  void Draw();
  void SetIntensity(double i);

private:
  const sfl::BubbleList * _bubble_list;
  mode_t _mode;
  double _intensity;

  void DrawFull();
  void DrawReduced();
  void DrawSnapped();
};

#endif // BLDRAWING_H