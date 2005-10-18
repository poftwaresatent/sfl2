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


#include "BubbleFactory.hpp"


using namespace std;



namespace sfl {



BubbleFactory::
BubbleFactory(int red,
	      int yellow,
	      int green):
  _top(0),
  _red(red),
  _yellow(yellow),
  _green(green),
  _level(0),
  _total(0)
{
  // create until green level
  while(_level < _yellow){
    Produce();
  }
}



BubbleFactory::
~BubbleFactory()
{
  while(_top != 0){
    Bubble *tmp = _top;
    Pop();
    delete tmp;
  }
}



void BubbleFactory::
SetRed(int i)
{
  _red = i;
}



void BubbleFactory::
SetYellow(int i)
{
  _yellow = i;
}



void BubbleFactory::
SetGreen(int i)
{
  _green = i;
}



Bubble * BubbleFactory::
New(double cutoffDistance,
    double xpos,
    double ypos)
{
  Bubble *tmp = Pop();

  if(tmp)
    tmp->Configure(cutoffDistance, pair<double, double>(xpos, ypos));
  
  return tmp;
}



Bubble* BubbleFactory::
Clone(Bubble * bubble)
{
  Bubble * tmp = Pop();

  if(tmp)
    tmp->CopyConstruct(*bubble);
  
  return tmp;
}



void BubbleFactory::
Delete(Bubble * bubble)
{
  Push(bubble);
}



void BubbleFactory::
Produce()
{
  Push(new Bubble());
  _total++;
}



void BubbleFactory::
EmulatedThread()
{
  static int entries(0);

  int skip;
  if(_level <= _red){
    skip = REDSKIP;
  }
  else if(_level <= _yellow){
    skip = YELLOWSKIP;
  }
  else if(_level <= _green){
    skip = GREENSKIP;
  }
  else{
    return;
  }

  if(entries < skip){
    entries++;
    return;
  }

  entries = 0;
  Produce();
}



void BubbleFactory::
Push(Bubble * bubble)
{
  // just a little paranoid...
  if(bubble == 0)
    return;

  bubble->_previous = _top;
  _top = bubble;    
  
  _level++;
}



Bubble* BubbleFactory::
Pop()
{
  if(_top == 0)
    return 0;
  
  Bubble * tmp = _top;

  _top = _top->_previous;
  _level--;
  
  return tmp;
}



}
