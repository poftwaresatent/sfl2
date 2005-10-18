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


#ifndef SUNFLOWER_BUBBLEFACTORY_HPP
#define SUNFLOWER_BUBBLEFACTORY_HPP



#include <sfl/bband/Bubble.hpp>



namespace sfl {



class BubbleFactory{
public:
  static const int DEFAULTRED    = 50;//20;
  static const int DEFAULTYELLOW = 100;//35;
  static const int DEFAULTGREEN  = 200;//70;
  
  BubbleFactory(int red    = DEFAULTRED,
		int yellow = DEFAULTYELLOW,
		int green  = DEFAULTGREEN);
  virtual ~BubbleFactory();
  
  void EmulatedThread();

  void SetRed(int i);
  void SetYellow(int i);
  void SetGreen(int i);

  Bubble * New(double cutoffDistance, double xpos, double ypos);
  Bubble * Clone(Bubble * bubble);
  void Delete(Bubble * bubble);



private:
  friend class BubbleList;	// hax for BubbleList copy constructor

  static const int REDSKIP    = 1;
  static const int YELLOWSKIP = 3;
  static const int GREENSKIP  = 7;

  Bubble * _top;

  int _red, _yellow, _green;
  int _level, _total;

  void Produce();

  void Push(Bubble * bubble);
  Bubble * Pop();
};


}


#endif // SUNFLOWER_BUBBLEFACTORY_HPP
