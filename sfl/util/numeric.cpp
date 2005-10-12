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


#include "numeric.hpp"
#include <cmath>


namespace sfl {


  int QuadraticEquation(double a,
			double b,
			double c,
			double & x1,
			double & x2)
  {
  if(absval(a) < epsilon){
    if(absval(b) < epsilon)
      return 0;

    x1 = - c / b;
    return 1;
  }
  
  if(absval(b) < epsilon){
    double sq(- c / a);
    if(sq < 0)
      return 0;

    if(absval(sq) < epsilon){
      x1 = 0;
      return 1;
    }

    x1 = sqrt(sq);
    x2 = - x1;
    return 2;
  }

  if(absval(c) < epsilon){
    x1 = 0;
    x2 = - b / a;
    return 2;
  }

  double root(b * b - 4 * a * c);
  if(root < 0)
    return 0;
  
  if(absval(root) < epsilon * epsilon){
    x1 = - 0.5 * b / a;
    return 1;
  }
  
  root = sqrt(root);
  x1 = - 0.5 * (b + root) / a;
  x2 = - 0.5 * (b - root) / a;
  return 2;
  }

}
