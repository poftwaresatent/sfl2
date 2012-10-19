/* -*- mode: C++; tab-width: 2 -*- */
/* 
 * Copyright (C) 2006
 * Swiss Federal Institute of Technology, Zurich. All rights reserved.
 * 
 * Developed at the Autonomous Systems Lab.
 * Visit our homepage at http://www.asl.ethz.ch/
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

#ifndef NPM_SMART_HPP
#define NPM_SMART_HPP


#include <npm/asl/AslBot.hpp>


namespace asl {
	class AckermannAlgorithm;
	class ArcControl;
	class AckermannController;
	class NavFuncQuery;
}


class Smart
  : public AslBot
{
private:
  Smart(const Smart &);
  
public:
  Smart(boost::shared_ptr<npm::RobotDescriptor> descriptor,
				const npm::World & world);
	
	virtual const asl::trajectory_t * GetTrajectory() const;	
	virtual bool GetRefpoint(asl::path_point &ref_point) const;
	
	/** \note Can return null. */
	const asl::ArcControl * GetArcControl() const;
	
	/** \note Can return null. */
	boost::shared_ptr<const asl::NavFuncQuery> GetQuery() const;
	
  
protected:
	boost::shared_ptr<asl::AckermannAlgorithm> m_ackalgo;
	boost::shared_ptr<const asl::AckermannController> m_acntrl;

	virtual
	void InitAlgorithm(boost::shared_ptr<npm::RobotDescriptor> descriptor,
										 smartparams const & params,
										 double carrot_distance,
										 double carrot_stepsize,
										 size_t carrot_maxnsteps,
										 double replan_distance,
										 std::string const & traversability_file,
										 int estar_step,
										 double wavefront_buffer,
										 std::string const & goalmgr_filename,
										 bool swiped_map_update,
										 double max_swipe_distance,
										 boost::shared_ptr<estar::AlgorithmOptions> estar_options,
										 boost::shared_ptr<asl::travmap_grow_options> grow_options,
										 bool estar_grow_grid,
										 /** \todo XXX aarghhhhh! */
										 double & robot_radius);
	
	virtual void InitScanners(boost::shared_ptr<sfl::Multiscanner> mscan,
														smartparams const & params);
	
	virtual void InitDrive(smartparams const & params);

	virtual void InitBody(smartparams const & params);
	
	virtual void MoreGraphics(std::string const & name,
														npm::World const & world,
														bool slow_drawing_enabled);
};

#endif // NPM_SMART_HPP
