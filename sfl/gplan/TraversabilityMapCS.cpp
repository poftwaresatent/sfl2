#include <iostream>
#include <math.h>
#include <sfl/gplan/GridFrame.hpp>
#include "TraversabilityMapCS.hpp"

using namespace sfl;
using namespace std;
using namespace estar;

TraversabilityMapCS::
TraversabilityMapCS(Frame &origin, 
										boost::shared_ptr<estar::Sprite> &sprite, 
										double resolution, double xSize, double ySize)
	: TraversabilityMap(origin, resolution, xSize,  ySize),
		sprite_(sprite)
{
			references_.reset(new array2d<Sprite::indexlist_t> (rint(xSize/resolution), rint(ySize/resolution)) ); 
}

bool TraversabilityMapCS::
SetValue(double global_x, double global_y, int & value)
{
	cout << "Todo: SetValue not defined yet for Configuration Space Maps." <<endl
			 << "For now use SetFree(x, y) and SetObst(x, y)" << endl; 
	return true;
}


bool TraversabilityMapCS::
SetObst(double global_x, double global_y)
{
	//cout << "  ***  " <<endl;
	//cout << "Set obstacle at " << global_x << " " << global_y << endl;
	
	gframe.From(global_x, global_y);

	//cout << "After gframe.From "  << global_x << " " << global_y << endl;

	Region *region = new Region(sprite_, global_x, global_y, data->xsize, data->ysize);
	Sprite::indexlist_t csMask = region->GetArea();

	const GridFrame::index_t idx(gframe.GlobalIndex(global_x, global_y));

	for (unsigned int i(0); i<csMask.size(); i++)
		{
			
			// update references
			Sprite::sindex ref(idx.v0, idx.v1 , 1.0 - csMask.at(i).r);
	
			GridFrame::index_t csidx;
			csidx.v0 = csMask.at(i).x;
			csidx.v1 = csMask.at(i).y;
			
			//cout << " affects grid cell " << csMask.at(i).x << " " << csMask.at(i).y << endl;

			if (references_->ValidIndex(csidx))
				(*references_)[csidx].push_back(ref);
	
			// update map
			if(data->ValidIndex(csidx))
				(*data)[csidx] = obstacle;//doubleToInt(1.0 - csMask.at(i).r);
		}
	return true;
}

bool TraversabilityMapCS::
SetFree(double global_x, double global_y)
{
	gframe.From(global_x, global_y);
	
	Region *region = new Region(sprite_, global_x, global_y, data->xsize, data->ysize);
	Sprite::indexlist_t csMask = region->GetArea();

	const GridFrame::index_t idx(gframe.GlobalIndex(global_x, global_y));

	for (unsigned int i(0); i<csMask.size(); i++)
		{
			GridFrame::index_t csidx;
			csidx.v0 = csMask.at(i).x;
			csidx.v1 = csMask.at(i).y;

			for (unsigned int j(0);j<((*references_)[csidx].size());j++)
				{
					// delete reference to parent cell
					if (((*references_)[csidx].at(j).x == idx.v0) 
							&& ((*references_)[csidx].at(j).y == idx.v1))
						(*references_)[csidx].erase((*references_)[csidx].begin()+j);
				}

			double max(0.0);

			for (int j(0);j<((*references_)[csidx].size());j++)
				{
					//get remaining max value 
					if (max < ((*references_)[csidx].at(j).r)) max = ((*references_)[csidx].at(j).r); 
				}			

			// update map
			if(data->ValidIndex(csidx))
				(*data)[csidx] = doubleToInt(max);
		}
	
	return true;
}

inline int TraversabilityMapCS::
doubleToInt(const double &val)
{
	return rint((obstacle-freespace)*val);
}

