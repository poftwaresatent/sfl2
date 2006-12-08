/* 
 * Copyright (C) 2006
 * Swiss Federal Institute of Technology, Zurich. All rights reserved.
 * 
 * Author: Roland Philippsen <roland dot philippsen at gmx dot net>
 *         Autonomous Systems Lab <http://www.asl.ethz.ch/>
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


#include "phelp.hpp"
#include <iostream>

using namespace std;

namespace poster {

  
  POSTER_ID create(const char * name, int length)
  {
    POSTER_ID id;
    if(OK != posterCreate(const_cast<char *>(name), length, &id)){
      cerr << "ERROR posterCreate() failed for ";
      h2perror(const_cast<char *>(name));
      return 0;
    }
    if( ! init(id, length)){
      cerr << "ERROR poster::init() failed for " << name << "\n";
      if(OK != posterDelete(&id)){
	cerr << "WARNING posterCreate() failed for ";
	h2perror(const_cast<char *>(name));
      }
      return 0;
    }
    return id;
  }
  
  
  POSTER_ID find(const char * name, int length)
  {
    POSTER_ID id;
    if(OK != posterFind(const_cast<char *>(name), &id)){
      cerr << "ERROR posterFind() failed for ";
      h2perror(const_cast<char *>(name));
      return 0;
    }
    if( ! check(id, length)){
      cerr << "ERROR poster::check() failed for " << name << "\n";
      return 0;
    }
    return id;
  }
  
  
  bool init(POSTER_ID id, int length)
  {
    void * tmp(calloc(1, length));
    if( ! tmp){
      cerr << "ERROR out of memory\n";
      return false;
    }
    if(length != posterWrite(id, 0, tmp, length)){
      h2perror("ERROR posterWrite()");
      free(tmp);
      return false;
    }
    free(tmp);
    return true;
  }
  
  
  bool check(POSTER_ID id, int length)
  {
    int have;
    if(OK != posterIoctl(id, FIO_GETSIZE, &have)){
      h2perror("ERROR posterIoctl()");
      return false;
    }
    if(have != length){
      cerr << "ERROR poster has wrong length (" << have
	   << " instead of " << length << ")\n";
      return false;
    }
    return true;
  }
  
  
  bool write(POSTER_ID id, const void * poster, int length)
  {
    if(length != posterWrite(id, 0, const_cast<void *>(poster), length)){
      h2perror("ERROR posterWrite()");
      return false;
    }
    return true;
  }
  
  
  bool read(POSTER_ID id, void * poster, int length)
  {
    if(length != posterRead(id, 0, poster, length)){
      h2perror("ERROR posterRead()");
      return false;
    }
    return true;
  }
  
  
  void destroy(POSTER_ID id)
  {
    if(OK != posterDelete(id))
      h2perror("WARNING posterDelete()");
  }
  
}
