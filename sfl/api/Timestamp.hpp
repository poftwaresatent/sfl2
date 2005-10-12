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


namespace sfl {


  class HALProxy;


  /**
     Encapsulates the HAL's timestamp type and presents it through a
     useful object-oriented interface. Much is inlined for efficient
     use, especially in conjunction with the STL.
  */
  class Timestamp
  {
  public:
    /** Default Timestamp with all zeros. */
    Timestamp() {
      m_stamp.tv_sec = 0;
      m_stamp.tv_nsec = 0;
    }
    
    /**
       Converts a HAL timestamp into a Timestamp instance. If you need
       a Timestamp of "unspecified" time, use Last() or First().
    */
    Timestamp(const struct timespec & stamp)
      : m_stamp(stamp) { }
    
    /**
       Sets the provided timespec to the current time.
       
       \return 0 on success.
    */
    static int Now(HALProxy & halProxy,
		   struct timespec * spec);
    
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
    
    /** Assignement operator. */
    Timestamp & operator = (const Timestamp & original) {
      m_stamp = original.m_stamp;
      return * this;
    }
    
    /**
       Dereference operator. You can use a Timestamp instance instead
       of a struct timespec, ie Timestamp behaves like a pointer to a
       <code>struct timespec</code> in certain circumstances.
    */
    struct timespec operator * () const { return m_stamp; }
    
    /**
       Ouput operator for human-readable messages, prints the
       Timestamp as a floating point number in seconds.
       
       \todo Is the field width reset to old after this call?
    */
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
	 cerr << id->first << ": " << id->second << endl;
       \endcode
    */
    friend bool operator < (const Timestamp & left, const Timestamp & right)
    { return
	(   left.m_stamp.tv_sec  <  right.m_stamp.tv_sec ) ||
	( ( left.m_stamp.tv_sec  == right.m_stamp.tv_sec ) &&
	  ( left.m_stamp.tv_nsec <  right.m_stamp.tv_nsec )   ); }
    
    /** The opposite of Timestamp::operator<(). */
    friend bool operator > (const Timestamp & left, const Timestamp & right)
    { return
	(   left.m_stamp.tv_sec  >  right.m_stamp.tv_sec ) ||
	( ( left.m_stamp.tv_sec  == right.m_stamp.tv_sec ) &&
	  ( left.m_stamp.tv_nsec >  right.m_stamp.tv_nsec )   ); }
    
    /** Equality operator. */
    friend bool operator == (const Timestamp & left, const Timestamp & right)
    { return
	( left.m_stamp.tv_sec  == right.m_stamp.tv_sec ) &&
	( left.m_stamp.tv_nsec == right.m_stamp.tv_nsec ); }
    
    /** Decrement operator. */
    Timestamp & operator -= (const Timestamp & other) {
      if(other.m_stamp.tv_nsec > m_stamp.tv_nsec){
	--m_stamp.tv_sec;
	m_stamp.tv_nsec += 1000000000;
      }
      m_stamp.tv_sec  -= other.m_stamp.tv_sec;
      m_stamp.tv_nsec -= other.m_stamp.tv_nsec;
      return * this;
    }
    
    /**
       Functor for using as sort key in STL containers. For
       example, you can create a set of timestamps that is
       automatically sorted in ascending chronological order like
       this:
       
       \code
       typedef std::set<Timestamp, Timestamp::less> chronology_t;
       chronology_t chrono;
       while(something())
         chrono.insert(RandomTimestamp());
       // print the random timestamps in ascending order:
       for(chronology_t::iterator ic(chrono.begin(); ic != chrono.end(); ++ic)
         cerr << * ic << endl;
       \endcode
    */
    class less {
    public:
      /** for references */
      bool operator () (const Timestamp & left, const Timestamp & right) const
      {return left < right; }
      
      /** for pointers */
      bool operator () (const Timestamp * left, const Timestamp * right) const
      { return (*left) < (*right); }
    };
    
  private:
    struct timespec m_stamp;
  };
  
}

#endif // SUNFLOWER_TIMESTAMP_HPP
