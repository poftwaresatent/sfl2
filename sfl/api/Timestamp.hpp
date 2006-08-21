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


#ifndef SUNFLOWER_TIMESTAMP_HPP
#define SUNFLOWER_TIMESTAMP_HPP


#include <iosfwd>


/* See comments in HAL.hpp */
struct timespec;


namespace sfl {


  /**
     Encapsulates timestamps (i.e. from HAL). Much is inlined for
     efficient use, especially in conjunction with the STL.
     
     \note By relying on HAL for timestamps, it is possible to
     "freeze" sunflower execution in simulation, which is extremely
     useful when debugging those fancy algorithms.
  */
  class Timestamp
  {
  public:
    /** Default Timestamp with all zeros. */
    Timestamp();
    
    /**
       Converts a HAL timestamp into a Timestamp instance. If you need
       a Timestamp of "unspecified" time, use Last() or First().
    */
    explicit Timestamp(const struct ::timespec & stamp);
    
    /**
       \return A (static) Timestamp of the last representable
       moment. This is useful for initializing a Timestamp before
       finding a minimum.
    */
    static const Timestamp & Last();
    
    /**
       \return A (static) Timestamp of the first representable
       moment. This is useful for initializing a Timestamp before
       finding a maximum.
    */
    static const Timestamp & First();
    
    /** Conversion operator. */
    Timestamp & operator = (const struct ::timespec & original);
    
    /** Prints the Timestamp as "seconds.nanoseconds". */
    friend std::ostream & operator << (std::ostream & os, const Timestamp & t);
    
    /**
       Basic comparison operator so that you can write code like this:
       
       \code
       Timestamp t0(some_data.GetTimestamp());
       Timestamp t1(other_data.GetTimestamp());
       if(t0 < t1)
         remove_old_data(some_data);
       else
         remove_old_data(other_data);
       \endcode

       It is also important for creating chronological maps of data,
       for example in order to implement a sensor history using STL:

       \code
       typedef std::multimap<Timestamp, double> distance_history_t;
       distance_history_t disthist;
       while(Whatever())
         disthist.insert(make_pair(Timestamp::Now(), ReadDistanceSensor()));
       // print readings in reverse:
       for(distance_history_t::reverse_iterator id(disthist.rbegin());
           id != disthist.rend();
	   ++id)
	 cerr << id->first << ": " << id->second << "\n";
       \endcode
    */
    friend bool operator < (const Timestamp & left, const Timestamp & right);
    
    /** The opposite of Timestamp::operator<(). */
    friend bool operator > (const Timestamp & left, const Timestamp & right);
    
    /** Equality operator. */
    friend bool operator == (const Timestamp & left, const Timestamp & right);
    
    /** Decrement operator. */
    Timestamp & operator -= (const Timestamp & other);
    
  private:
    struct ::timespec m_stamp;
  };

}

#endif // SUNFLOWER_TIMESTAMP_HPP
