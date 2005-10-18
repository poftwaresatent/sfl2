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


#include "Lookup.hpp"
#include <sfl/util/numeric.hpp>
#include <cstdlib>
#include <iostream>


using std::cerr;


namespace sfl {


  int Lookup::dimension(0);
  double ** Lookup::buffer(0);
  double * Lookup::histogram(0);
  Lookup::bin_t Lookup::bin[maxNbins + 1]; // attention: +1 !!!
  int Lookup::nBins(maxNbins);


  Lookup::
  Lookup(int _dimension,
	 double minValidValue,
	 double maxValidValue):
    minValid(minValidValue),
    maxValid(maxValidValue),
    quantizer(0)
  {
    if(dimension == 0)
      dimension = _dimension;
    else if(dimension != _dimension){
      cerr << "ERROR in Lookup::Lookup()\n"
	   << "   dimension previously given as " << dimension << "\n"
	   << "   and now given as " << _dimension << "\n"
	   << "   future versions should become more flexible in this regard.\n";
      abort();
    }
  
    if(buffer == 0){
      buffer = new double*[dimension];
      for(int i = 0; i < dimension; ++i)
	buffer[i] = new double[dimension];
    }

    if(histogram == 0)
      histogram = new double[dimension * dimension];

    value = new unsigned short*[dimension];
    for(int i = 0; i < dimension; ++i)
      value[i] = new unsigned short[dimension];
  }


  Lookup::
  ~Lookup()
  {
    if(quantizer != 0)
      delete[] quantizer;

    if(buffer != 0){
      for(int i = 0; i < dimension; ++i)
	delete[] buffer[i];
      delete[] buffer;
      buffer = 0;
    }

    if(histogram != 0){
      delete[] histogram;
      histogram = 0;
    }

    for(int i = 0; i < dimension; ++i)
      delete[] value[i];
    delete[] value;
  }


  void Lookup::
  LoadBuffer(int iqdl,
	     int iqdr,
	     double value)
  {
    buffer[iqdl][iqdr] = value;
  }


  void Lookup::
  SaveBuffer()
  {
    CreateHistogram();
    LloydMax();
    Quantize();
    
    static const bool dump_whole_table(false);
    if(dump_whole_table){
      cerr << "INFO from Lookup::SaveBuffer():\n  table:\n";
      for(int i(0); i < dimension; ++i){
	cerr << "   ";
	for(int j(0); j < dimension; ++j){
	  const double val(Get(i, j));
	  if(val >= 0)
	    fprintf(stderr, " %3.1f", val);
	  else
	    cerr << " inv";
	}
	cerr << "\n";
      }
    }
    
    static const bool dump_compression_statistics(false);
    if(dump_compression_statistics){
      double diffmin(std::numeric_limits<double>::max());
      double diffmax(std::numeric_limits<double>::min());
      double diffsum(0);
      for(int i(0); i < dimension; ++i)
	for(int j(0); j < dimension; ++j){
	  const double diff(Get(i, j) - buffer[i][j]);
	  if(diff < diffmin) diffmin = diff;
	  if(diff > diffmax) diffmax = diff;
	  diffsum += diff;
	}
      cerr << "INFO from Lookup::SaveBuffer():\n"
	   << "  diffmin  = " << diffmin << "\n"
	   << "  diffmax  = " << diffmax << "\n"
	   << "  diffmean = " << diffsum / sqr(dimension) << "\n";
    }
  }


  double Lookup::
  Get(int iqdl,
      int iqdr)
    const
  {
    if(noValid)
      return -1;
    return quantizer[value[iqdl][iqdr]];
  }


  void Lookup::
  CreateHistogram()
  {
    nBins = maxNbins;

    // load buffer into histogram
    int count(0);
    for(int i = 0; i < dimension; ++i)
      for(int j = 0; j < dimension; ++j){
	histogram[count] = buffer[i][j];
	++count;
      }

    // sort histogram (insert sort, simple but slow)
    for(int j = 1; j < dimension * dimension; ++j){
      double key(histogram[j]);
      int i(j - 1);

      while((i >= 0) && (histogram[i] > key)){
	histogram[i + 1] = histogram[i];
	--i;
      }

      histogram[i + 1] = key;
    }
  }


  void Lookup::
  LloydMax()
  {
    // initialize bins
    double vmax;
    bool found(false);
    for(int i = 0; i < dimension * dimension; ++i)
      if(histogram[i] <= maxValid){
	found = true;
	vmax = histogram[i];
      }
      else
	break;

    if( ! found){
      noValid = true;
      return;
    }
    noValid = false;

    double vmin;
    // there will be at least one valid value, because of the previous check
    // (if maxValid > minValid)
    for(int i = 0; i < dimension * dimension; ++i)
      if(histogram[i] >= minValid){
	vmin = histogram[i];
	break;
      }

    // okay, this could be done faster...
    double scale((vmax - vmin) / maxNbins);
    for(int i = 0; i < maxNbins; ++i){
      bin[i].t = vmin + i * scale;
      bin[i].r = -1;
    }
    bin[maxNbins].t = vmax;

    // iterate
    nBins = maxNbins;
    bool finished(false);
    while( ! finished){
      finished = true;

      // calculate levels
      double r2;
      for(int i = 0; i < nBins - 1; ++i){
	r2 = PartialMean(bin[i].t, bin[i + 1].t);
	if(absval(bin[i].r - r2) > 1e-9) // hax: hardcoded epsilon
	  finished = false;
	bin[i].r = r2;
      }
      r2 = PartialMeanInclusive(bin[nBins - 1].t, bin[nBins].t);
      if(absval(bin[nBins - 1].r - r2) > 1e-9) // hax: hardcoded epsilon
	finished = false;
      bin[nBins - 1].r = r2;

      // remove empty bins
      for(int i = 0; i < nBins; ++i)
	if(bin[i].r < 0){
	  --nBins;
	  for(int j = i; j < nBins; ++j){
	    bin[j].t = bin[j + 1].t;
	    bin[j].r = bin[j + 1].r;
	  }
	  bin[nBins].t = bin[nBins + 1].t; // one more boundary than levels
	  --i;			// should never happen that bin[0].r < 0...
	}

      // calculate boundaries
      for(int i = 1; i < nBins; ++i)
	bin[i].t = 0.5 * (bin[i - 1].r + bin[i].r);
    } //   while(!finished)
  }


  void Lookup::
  Quantize()
  {
    if(quantizer != 0){
      delete[] quantizer;
      quantizer = 0;
    }

    if(noValid)
      return;

    quantizer = new double[nBins + 1]; // one special bin for "no intersection"

    // init quantizer values
    for(int i = 0; i < nBins - 1; ++i)
      quantizer[i] = PartialMin(bin[i].t, bin[i + 1].t);
    quantizer[nBins - 1] = PartialMinInclusive(bin[nBins - 1].t, bin[nBins].t);
    quantizer[nBins] = -1;

    // quantize buffer and store it in value[][]
    for(int i = 0; i < dimension; ++i){
      for(int j = 0; j < dimension; ++j){
	double val(buffer[i][j]);
	int k;
	for(k = 0; k < nBins; ++k)
	  if((val >= bin[k].t) && (val < bin[k + 1].t)){
	    value[i][j] = k;
	    break;
	  }
	if(k == nBins){
	  if(val == bin[nBins].t){ // last bin is "right-inclusive"
	    value[i][j] = nBins - 1;
	  }
	  else{
	    value[i][j] = nBins; // no intersection (or too far)
	  }
	}
      }
    }
  }


  double Lookup::
  PartialMean(double from,
	      double to)
  {
    double sum(0);
    double count(0);

    for(int i = 0; i < dimension * dimension; ++i)
      if(histogram[i] >= from){
	if(histogram[i] < to){
	  sum += histogram[i];
	  count += 1;
	}
	else
	  break;
      }
  
    if(count < 1)
      return -1;

    return sum / count;
  }


  double Lookup::
  PartialMeanInclusive(double from,
		       double to)
  {
    double sum(0);
    double count(0);

    for(int i = 0; i < dimension * dimension; ++i)
      if(histogram[i] >= from){
	if(histogram[i] <= to){
	  sum += histogram[i];
	  count += 1;
	}
	else
	  break;
      }
  
    if(count < 1)
      return -1;

    return sum / count;
  }


  double Lookup::
  PartialMin(double from,
	     double to)
  {
    for(int i = 0; i < dimension * dimension; ++i)
      if((histogram[i] >= from) &&
	 (histogram[i] < to))
	return histogram[i];

    cerr << "ERROR in Lookup::PartialMin(" << from
	 << ", " << to << ")\n   no minimum found\n";
    abort();
    return 0;
  }


  double Lookup::
  PartialMinInclusive(double from,
		      double to)
  {
    for(int i = 0; i < dimension * dimension; ++i)
      if((histogram[i] >= from) &&
	 (histogram[i] <= to))
	return histogram[i];

    cerr << "ERROR in Lookup::PartialMinInclusive(" << from
	 << ", " << to << ")\n   no minimum found\n";
    abort();
    return 0;
  }

}
