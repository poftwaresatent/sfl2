#ifndef TRAVERSABILITY_MAP_CS
#define TRAVERSABILITY_MAP_CS

#include <estar/Sprite.hpp>
#include <estar/Region.hpp>

#include <sfl/gplan/TraversabilityMap.hpp>

namespace sfl{

	class TraversabilityMapCS
		: public TraversabilityMap
	{
	public:
		//TraversabilityMapCS();
	TraversabilityMapCS(Frame &origin, boost::shared_ptr<estar::Sprite> &sprite, double resolution, double xSize, double ySize); 

		//	~TraversabilityMapCS() { }
		// virtual functions
		bool SetValue(double global_x, double global_y, int & value);
		bool SetObst(double global_x, double global_y);
		bool SetFree(double global_x, double global_y);
	private:
		int doubleToInt(const double &val);
		boost::shared_ptr<array2d<estar::Sprite::indexlist_t> >  references_; /**< default NULL */
		boost::shared_ptr<estar::Sprite> sprite_;
	};

	
}
#endif
