#ifndef MAPPER2D_HPP
#define MAPPER2D_HPP

#include <sfl/api/Pose.hpp>
#include <sfl/api/Scanner.hpp>
#include <sfl/gplan/TraversabilityMap.hpp>
#include <sfl/gplan/TraversabilityMapCS.hpp>

class Mapper2d{
public:
	Mapper2d();
	/*	inline bool update(boost::shared_ptr<sfl::Pose> &odo , sfl::Scanner &scanner)
	{
		return update(odo->get(), scanner);
	}

	inline bool update(sfl::Pose &odo, boost::shared_ptr<sfl::Scanner> &scanner)
	{
		return update(odo, scanner->get());
	}

	bool update(boost::shared_ptr<sfl::Pose> &odo , boost::shared_ptr<sfl::Scanner> &scanner)
	{
		return update(odo->get(), scanner->get());
		}*/

	bool update(sfl::Pose &odo, sfl::Scanner &scanner);

	boost::shared_ptr<sfl::TraversabilityMap> getTravMap();

private:
	bool simpleCellUpdate(double x, double y);
	bool swipeCellUpdate(double x, double y);
	int map_obstacle_between_trace(int fx0, int fy0, int fx1, int fy1);

	boost::shared_ptr <sfl::TraversabilityMap> travMap_;
	sfl::Pose odo_;
};
#endif
