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


#include "Multiscanner.hpp"
#include <cmath>
#include <iostream>		// dbg


using boost::shared_ptr;
using namespace std;


namespace sfl {
  
  
  void Multiscanner::
  Add(shared_ptr<Scanner> scanner)
  {
    m_total_nscans += scanner->Nscans();
    m_scanner.push_back(scanner);
  }
  
  
  index_t Multiscanner::
  Nscanners()
    const
  {
    return m_scanner.size();
  }

  
  shared_ptr<Scanner> Multiscanner::
  GetScanner(index_t i)
    const
  {
    if((i < 0) || (i >= m_scanner.size()))
      return shared_ptr<Scanner>();
    return m_scanner[i];
  }
  
  
  shared_ptr<Scan> Multiscanner::
  CollectScans()
    const
  {
    shared_ptr<Scan> result(new Scan(m_total_nscans));
    
    size_t iOverall(0);		// incremented in nested loop
    for(size_t iScanner(0); iScanner < m_scanner.size(); ++iScanner){
      shared_ptr<Scanner> scanner(m_scanner[iScanner]);
      
      if(scanner->Tlower() < result->m_tlower)
	result->m_tlower = scanner->Tlower();
      if(scanner->Tupper() > result->m_tupper)
	result->m_tupper = scanner->Tupper();
      
      for(size_t iRay(0); iRay < scanner->Nscans(); ++iRay, ++iOverall){
	double x, y;
	if(scanner->GetLocal(iRay, x, y) == Scanner::SUCCESS){
	  result->m_data[iOverall].locx = x;
	  result->m_data[iOverall].locy = y;
	  result->m_data[iOverall].phi = atan2(y, x);
	  result->m_data[iOverall].rho = sqrt(x*x + y*y);
	}
      }
    }
    if(iOverall != m_total_nscans){
      cerr << "BUG in Multiscanner::CollectScans():\n"
	   << "  iOverall != m_total_nscans\n"
	   << "  iOverall == " << iOverall << "\n"
	   << "  m_total_nscans == " << m_total_nscans << "\n";
      abort();
    }
    
    return result;
  }
  
  
  shared_ptr<GlobalScan> Multiscanner::
  CollectGlobalScans(const Frame & position)
    const
  {
    return shared_ptr<GlobalScan>(new GlobalScan(CollectScans(), position));
  }
  
}
