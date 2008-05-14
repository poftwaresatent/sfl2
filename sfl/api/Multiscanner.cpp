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
#include "Scanner.hpp"
#include "Scan.hpp"
#include "Odometry.hpp"
#include "Pose.hpp"
#include <sfl/util/numeric.hpp>
#include <iostream>		// dbg
#include <cmath>


using namespace boost;
using namespace std;


namespace sfl {


#warning "why not a const Odometry?"
  Multiscanner::
  Multiscanner(shared_ptr<Odometry> odometry)
    : m_odometry(odometry)
  {
  }
  
  
  void Multiscanner::
  Add(shared_ptr<Scanner> scanner)
  {
    m_total_nscans += scanner->nscans;
    m_scanner.push_back(scanner);
  }
  
  
  size_t Multiscanner::
  Nscanners() const
  {
    return m_scanner.size();
  }

  
  shared_ptr<Scanner> Multiscanner::
  GetScanner(size_t i) const
  {
    if((i < 0) || (i >= m_scanner.size()))
      return shared_ptr<Scanner>();
    return m_scanner[i];
  }
  
  
  shared_ptr<Scan> Multiscanner::
  CollectScans() const
  {
    // initialize to zero size (just add VALID data) and with INVERTED
    // timestamps to detect the min and max actual ones
    shared_ptr<Scan>
      result(new Scan(0, Timestamp::Last(), Timestamp::First(),
		      *m_odometry->Get()));
    
    for(size_t iScanner(0); iScanner < m_scanner.size(); ++iScanner){
      shared_ptr<Scanner> scanner(m_scanner[iScanner]);
      
      if(scanner->Tlower() < result->tlower)
	result->tlower = scanner->Tlower();
      if(scanner->Tupper() > result->tupper)
	result->tupper = scanner->Tupper();
      
      for(size_t iRay(0); iRay < scanner->nscans; ++iRay){
	scan_data data;
	// note: only add valid data, not even OUT_OF_RANGE!
	if(scanner->GetData(iRay, data) == Scanner::SUCCESS){
	  data.phi = atan2(data.locy, data.locx);
	  data.rho = sqrt(sqr(data.locx) + sqr(data.locy));
	  data.in_range = true;
	  result->data.push_back(data);
	}
      }
    }
    
    return result;
  }
  
  
  boost::shared_ptr<Multiscanner::raw_scan_collection_t> Multiscanner::
  CollectRawScans() const
  {
    boost::shared_ptr<raw_scan_collection_t>
      result(new raw_scan_collection_t());
    for(size_t iScanner(0); iScanner < m_scanner.size(); ++iScanner){
      shared_ptr<Scanner> scanner(m_scanner[iScanner]);
      result->push_back(scanner->GetScanCopy());
    }
    return result;
  }
  
  
#warning "remove this method, and kick out the iostream include directive"
  size_t Multiscanner::
  ComputeOffset(boost::shared_ptr<const Scanner> scanner) const
  {
    size_t off(0);
    for(vector_t::const_iterator is(m_scanner.begin());
	is != m_scanner.end();
	++is){
      if(scanner == *is)
	return off;
      off += (*is)->nscans;
    }
    std::cerr << "WARNING in Multiscanner::ComputeOffset():\n"
	      << "  scanner not registered, returning 0\n";
    return 0;
  }


  bool Multiscanner::
  UpdateAll()
  {
    bool ok(true);
    for(size_t ii(0); ii < m_scanner.size(); ++ii)
      if(0 > m_scanner[ii]->Update())
	ok = false;
    return ok;
  }

}
