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


using std::make_pair;


namespace sfl {
  
  
  BubbleFactory::
  BubbleFactory(int red,
		int yellow,
		int green):
    m_top(0),
    m_red(red),
    m_yellow(yellow),
    m_green(green),
    m_level(0),
    m_total(0)
  {
    Produce(m_yellow);
  }
  
  
  BubbleFactory::
  ~BubbleFactory()
  {
    while(m_top != 0){
      Bubble *tmp = m_top;
      Pop();
      delete tmp;
    }
  }
  
  
  void BubbleFactory::
  SetRed(int i)
  {
    m_red = i;
  }
  
  
  void BubbleFactory::
  SetYellow(int i)
  {
    m_yellow = i;
  }
  
  
  void BubbleFactory::
  SetGreen(int i)
  {
    m_green = i;
  }
  
  
  Bubble * BubbleFactory::
  New(double cutoffDistance,
      double xpos,
      double ypos)
  {
    Bubble *tmp = Pop();
    if(tmp)
      tmp->Configure(cutoffDistance, make_pair(xpos, ypos));
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
  Produce(int batch)
  {
    for(/**/; batch > 0; --batch){
      Push(new Bubble());
      m_total++;
    }
  }
  
  
  void BubbleFactory::
  EmulatedThread()
  {
    static int entries(0);
    
    int skip, batch;
    if(m_level <= m_red){
      skip = REDSKIP;
      batch = REDBATCH;
    }
    else if(m_level <= m_yellow){
      skip = YELLOWSKIP;
      batch = YELLOWBATCH;
    }
    else if(m_level <= m_green){
      skip = GREENSKIP;
      batch = GREENBATCH;
    }
    else
      return;
    
    if(entries < skip){
      entries++;
      return;
    }
    
    entries = 0;
    Produce(batch);
  }
  
  
  void BubbleFactory::
  Push(Bubble * bubble)
  {
    if(bubble == 0)
      return;
    bubble->_previous = m_top;
    m_top = bubble;    
    m_level++;
  }
  
  
  Bubble* BubbleFactory::
  Pop()
  {
    if(m_top == 0)
      return 0;
    Bubble * tmp = m_top;
    m_top = m_top->_previous;
    m_level--;
    return tmp;
  }
  
}
