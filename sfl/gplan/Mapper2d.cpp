#include <sfl/api/Scan.hpp>
#include <estar/Sprite.hpp>
#include <iostream>

#include "Mapper2d.hpp"

using namespace sfl;
using namespace std;
using namespace estar;

Mapper2d::Mapper2d()
{
	/*! todo get frame transform as parameters (from config file)*/ 
	Frame *travMapOrigin = new Frame(-10.0 ,-60.0 ,0);
	
	boost::shared_ptr<Sprite> sprite(new Sprite(1.0, 0.5));
	
	/*! todo get map size as parameters (from config file)*/ 
	travMap_.reset(new TraversabilityMapCS(*travMapOrigin 
																				 , sprite
																				 , 0.5, 100.0, 100.0));
}

boost::shared_ptr<TraversabilityMap> Mapper2d::getTravMap()
{
	return travMap_;
}


/*! Update of traversability Map based on Scan */
bool Mapper2d::update(Pose &odo , Scanner &scanner)
{
	////	cerr << "==================================================\n";
	scan_data p;
	for (size_t i(0); i< scanner.nscans; i++)
		{
			/* get scan point */
			scanner.GetData(i, p);
			
			if(p.rho >= scanner.rhomax)
				continue;
			
			/* translate coordinates to global */
			double x= p.locx;
			double y= p.locy;
			odo.To(x, y);

			////			cerr << "DBG " << x << "   " << y << "\n";

			/* update traversability map */
			simpleCellUpdate(x, y);
		}
	odo_ = odo;

	return true;
}

// Private functions

inline bool Mapper2d::simpleCellUpdate(double x, double y)
{
	travMap_->SetObst(x, y);
	return true; 
}

bool Mapper2d::swipeCellUpdate(double x, double y)
{
	
	double x0(odo_.X());
	double y0(odo_.Y());

	const GridFrame::index_t idx0(travMap_->gframe.GlobalIndex(x0, y0));
	const GridFrame::index_t idx(travMap_->gframe.GlobalIndex(x, y));

	map_obstacle_between_trace(idx0.v0, idx0.v1, idx.v0, idx.v1);

	travMap_->SetObst(x, y);
	
	return true;
}

  /* run Bresenham's line drawing algorithm, adapted from:
         www.cs.nps.navy.mil/people/faculty/capps/iap/class1/lines/lines.html 
*/
int Mapper2d::map_obstacle_between_trace(int fx0, int fy0, int fx1, int fy1){

   int x0 = fx0, y0 = fy0, x1 = fx1, y1 = fy1;

	 GridFrame::index_t idx(travMap_->gframe.GlobalIndex(x0, y0));

   int dy = y1 - y0;
   int dx = x1 - x0;
   int stepx, stepy;
   int fraction;

   if (dy < 0) { dy = -dy;  stepy = -1; } else { stepy = 1; }
   if (dx < 0) { dx = -dx;  stepx = -1; } else { stepx = 1; }
   dy <<= 1;                            /* dy is now 2*dy*/
   dx <<= 1;                            /* dx is now 2*dx*/

	 idx.v0 = x0;
	 idx.v1 = y0;
	 if(travMap_->data->ValidIndex(idx))
		 if ((*travMap_->data)[idx] == travMap_->obstacle)
			 {
				 (*travMap_->data)[idx] = travMap_->freespace;
			 }

   if (dx > dy) {
     fraction = dy - (dx >> 1);                 /* same as 2*dy - dx */
     while (x0 != x1) {
       if (fraction >= 0) {
				 y0 += stepy;
				 fraction -= dx;                 /* same as fraction -= 2*dx */
			 }
       x0 += stepx;
       fraction += dy;                   /* same as fraction -= 2*dy */
			 
			 idx.v0 = x0;
			 idx.v1 = y0;
			 if(travMap_->data->ValidIndex(idx))
				 if ((*travMap_->data)[idx] == travMap_->obstacle)
					 {
						 (*travMap_->data)[idx] = travMap_->freespace;
					 }
		 }
		 idx.v0 = x0;
		 idx.v1 = y0;
		 if(travMap_->data->ValidIndex(idx))
			 if ((*travMap_->data)[idx] == travMap_->obstacle)
				 {
					 (*travMap_->data)[idx] = travMap_->freespace;
				 }
   } 
   else {
     fraction = dx - (dy >> 1);
     while (y0 != y1) {
       if (fraction >= 0) {
				 x0 += stepx;    
				 fraction -= dy;
       }
       y0 += stepy;
       fraction += dx;
			 
		 idx.v0 = x0;
		 idx.v1 = y0;
		 if(travMap_->data->ValidIndex(idx))
			 if ((*travMap_->data)[idx] == travMap_->obstacle)
				 {
					 (*travMap_->data)[idx] = travMap_->freespace;
				 }
		 
     }
		 
		 idx.v0 = x0;
		 idx.v1 = y0;
		 if(travMap_->data->ValidIndex(idx))
			 if ((*travMap_->data)[idx] == travMap_->obstacle)
				 {
					 (*travMap_->data)[idx] = travMap_->freespace;
				 }

   }
   return 0;
}
