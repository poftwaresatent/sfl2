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


#ifndef SUNFLOWER_OBJECTIVE_HPP
#define SUNFLOWER_OBJECTIVE_HPP


#include <iosfwd>


namespace sfl {
  
  
  class DynamicWindow;
  
  
  class Objective
  {
  public:
    Objective(const DynamicWindow & dynamic_window);
    virtual ~Objective();

    virtual void Initialize(std::ostream * progress_stream) = 0;

    // cannot be abstract because subclasses might need further
    // information, such as the current scan, or the robot pose.
    //     virtual void Calculate(unsigned int qdlMin,
    // 			   unsigned int qdlMax,
    // 			   unsigned int qdrMin,
    // 			   unsigned int qdrMax) = 0;

    void Rescale(unsigned int qdlMin,
		 unsigned int qdlMax,
		 unsigned int qdrMin,
		 unsigned int qdrMax);

    /** \pre all indices must be valid. */
    double Value(unsigned int qdlIndex,
		 unsigned int qdrIndex) const;

    /** \pre all indices must be valid. */
    double Min(unsigned int qdlMin,
	       unsigned int qdlMax,
	       unsigned int qdrMin,
	       unsigned int qdrMax) const;

    /** \pre all indices must be valid. */
    double Max(unsigned int qdlMin,
	       unsigned int qdlMax,
	       unsigned int qdrMin,
	       unsigned int qdrMax) const;

    inline double MinValue() const;
    inline double MaxValue() const;
    inline unsigned int Dimension() const;



  protected:
    static const double _minValue = 0;
    static const double _maxValue = 1;

    const DynamicWindow & _dynamic_window;
    const unsigned int _dimension;

    double ** _value;		//[dimension][dimension];
  };



  double Objective::
  MinValue()
    const
  {
    return _minValue;
  }


  double Objective::
  MaxValue()
    const
  {
    return _maxValue;
  }



  unsigned int Objective::
  Dimension()
    const
  {
    return _dimension;
  }

}

#endif // SUNFLOWER_OBJECTIVE_HPP
