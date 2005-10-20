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
    // initialize to zero size, just add VALID data
    shared_ptr<Scan> result(new Scan(0));
    
    for(size_t iScanner(0); iScanner < m_scanner.size(); ++iScanner){
      shared_ptr<Scanner> scanner(m_scanner[iScanner]);
      
      if(scanner->Tlower() < result->m_tlower)
	result->m_tlower = scanner->Tlower();
      if(scanner->Tupper() > result->m_tupper)
	result->m_tupper = scanner->Tupper();
      
      for(size_t iRay(0); iRay < scanner->Nscans(); ++iRay){
	double x, y;
	// note: only add valid data, not even OUT_OF_RANGE!
	if(scanner->GetLocal(iRay, x, y) == Scanner::SUCCESS){
	  Scan::data_t data;
	  data.phi = atan2(y, x);
	  data.rho = sqrt(x*x + y*y);
	  data.locx = x;
	  data.locy = y;
	  result->m_data.push_back(data);
	}
      }
    }
    
    return result;
  }
  
  
  shared_ptr<GlobalScan> Multiscanner::
  CollectGlobalScans(const Frame & position)
    const
  {
    return shared_ptr<GlobalScan>(new GlobalScan(CollectScans(), position));
  }
  
  
  size_t Multiscanner::
  ComputeOffset(boost::shared_ptr<const Scanner> scanner)
    const
  {
    size_t off(0);
    for(vector_t::const_iterator is(m_scanner.begin());
	is != m_scanner.end();
	++is){
      if(scanner == *is)
	return off;
      off += (*is)->Nscans();
    }
    std::cerr << "WARNING in Multiscanner::ComputeOffset():\n"
	      << "  scanner not registered, returning 0\n";
    return 0;
  }
  
}
