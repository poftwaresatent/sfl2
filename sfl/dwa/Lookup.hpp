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


#ifndef SUNFLOWER_LOOKUP_HPP
#define SUNFLOWER_LOOKUP_HPP



namespace sfl {



/**
   Compressed lookup tables, kind of hand-tailored for
   DistanceObjective.

   \note This class is a dinosaur, please refactor.
*/

class Lookup
{
public:
  Lookup(int dimension,
	 double minValidValue,
	 double maxValidValue);
  ~Lookup();

  static void LoadBuffer(int iqdl, int iqdr, double value);
  void SaveBuffer();
  double Get(int iqdl, int iqdr) const;

protected:
  typedef struct { double t, r; } bin_t;
  
  static const int maxNbins = 200; // < 2^sizeof(short) - 1
  static int dimension;
  
  static double ** buffer;
  static double * histogram;
  static bin_t bin[maxNbins + 1]; // attention: +1 !!!
  static int nBins;

  double minValid, maxValid;
  bool noValid;
  double * quantizer;
  unsigned short ** value;

  static void CreateHistogram();
  void LloydMax();
  void Quantize();
  static double PartialMean(double from, double to);
  static double PartialMeanInclusive(double from, double to);
  static double PartialMin(double from, double to);
  static double PartialMinInclusive(double from, double to);
};



}

#endif // SUNFLOWER_LOOKUP_HPP
