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
  BubbleFactory(int _batchsize)
    : batchsize(_batchsize)
  {
    Produce(batchsize);
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
  
  
  Bubble * BubbleFactory::
  New(double cutoffDistance, double xpos, double ypos)
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
  Push(Bubble * bubble)
  {
    if(bubble == 0)
      return;
    bubble->_previous = m_top;
    m_top = bubble;    
    ++m_level;
  }
  
  
  Bubble * BubbleFactory::
  Pop()
  {
    if(m_level == 0)
      Produce(batchsize);
    Bubble * tmp = m_top;
    m_top = m_top->_previous;
    --m_level;
    return tmp;
  }
  
}
