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


#ifndef SUNFLOWER_NUMERIC_HPP
#define SUNFLOWER_NUMERIC_HPP


#include <cmath>


namespace sfl {


  static const double epsilon = 1e-9;


  /**
     Evaluates to the minimum of it's two arguments.

     \todo Would have liked to use const references instead of values,
     but that creates problems for comparing against constants
     (e.g. things defined as "static const double"). This doesn't really
     matter for builtin-types, but comparing more complex types entails
     constructor and destructor calls...
  */
  template<typename T>
  T minval(T a, T b)
  {
    return a < b ? a : b;
  }


  /**
     Evaluates to the maximum of it's two arguments.
  */
  template<typename T>
  T maxval(T a, T b)
  {
    return a > b ? a : b;
  }
  
  
  /**
     Evaluates to the middle argument or one of the bounds if the
     value is too great or too small.
  */
  template<typename T>
  T boundval(T lower_bound, T value, T upper_bound)
  {
    return maxval(minval(value, upper_bound), lower_bound);
  }
  
  
  /**
     Evaluates to the absolute value of it's argument.
  */
  template<typename T>
  T absval(const T & a)
  {
    return a > 0 ? T(a) : T(-a);
  }


  /**
     Calculates the square of its argument.
  */
  template<typename T>
  T sqr(T x)
  {
    return x * x;
  }


  /**
     Set an angle to the range -pi < angle <= +pi
  */
  inline double mod2pi(double x)
  {
    x = fmod(x, 2 * M_PI);
    if(x > M_PI)
      x -= 2 * M_PI;
    else if(x <= - M_PI)
      x += 2 * M_PI;
    
    return x;
  }


  /**
     Solves the quadratic equation "a * x ^ 2 + b * x + c == 0".

     \note Handles special cases "gracefully" based on the properties
     (absval(a) < epsilon), (absval(b) < epsilon), (absval(c) <
     epsilon), and (absval(b*b-4*a*c) < epsilon*epsilon).

     \return The number of solutions (0, 1, or 2), it only sets x1 / x2
     for the returned number of solutions (e.g. if it returns 1, only x1
     contains a useful number).
  */
  int QuadraticEquation(double a, double b, double c,
			double & x1, double & x2);

}

#endif // SUNFLOWER_NUMERIC_HPP
