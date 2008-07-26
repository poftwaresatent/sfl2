/* -*- mode: C++; tab-width: 2 -*- */
/* 
 * Copyright (C) 2007
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

#include "MapperRefDrawing.hpp"
#include "wrap_gl.hpp"
#include "Manager.hpp"
#include <sfl/gplan/Mapper2d.hpp>
#include <sfl/util/pdebug.hpp>
#include <sfl/util/vec2d.hpp>
#include <sfl/util/Random.hpp>
#include <cmath>
#include <err.h>


#ifdef NPM_DEBUG
# define PDEBUG PDEBUG_OUT
#else // ! NPM_DEBUG
# define PDEBUG PDEBUG_OFF
#endif // NPM_DEBUG
#define PVDEBUG PDEBUG_OFF


using namespace sfl;
using namespace boost;
using namespace std;


namespace npm {
  
  
  MapperRefDrawing::
  MapperRefDrawing(const std::string & name,
									 boost::shared_ptr<const sfl::Mapper2d> mapper,
									 bool draw_link)
    : Drawing(name,
							"the references and backlinks in a sfl::Mapper2d",
							Instance<UniqueManager<Drawing> >()),
      m_mapper(mapper),
			m_draw_link(draw_link)
  {
  }
  

  void MapperRefDrawing::
  Draw()
  {
		try {
			const GridFrame & gframe(m_mapper->GetGridFrame());
			glMatrixMode(GL_MODELVIEW);
			glPushMatrix();
			glTranslated(gframe.X(), gframe.Y(), 0);
			glRotated(180 * gframe.Theta() / M_PI, 0, 0, 1);
			glScaled(gframe.Delta(), gframe.Delta(), 1);

			shared_ptr<RDTravmap> rdtravmap(m_mapper->CreateRDTravmap());
			vector<vec2d<double> > pts;
		
			ssize_t const xbegin(rdtravmap->GetXBegin());
			ssize_t const xend(rdtravmap->GetXEnd());
			ssize_t const ybegin(rdtravmap->GetYBegin());
			ssize_t const yend(rdtravmap->GetYEnd());
		
			if (m_draw_link) {
				const Mapper2d::linkmap_t & linkmap(m_mapper->GetLinkmap());
				glColor3d(1, 1, 0.5);
				glBegin(GL_LINES);
				for (ssize_t ix(xbegin); ix < xend; ++ix)
					for (ssize_t iy(ybegin); iy < yend; ++iy) {
						int val;
						if ( ! rdtravmap->GetValue(ix, iy, val))
							continue;
						const Mapper2d::link_t & link(linkmap.at(ix, iy));
						if (link.empty())
							continue;
						for (Mapper2d::link_t::const_iterator ii(link.begin());
								 ii != link.end(); ++ii) {
							vec2d<double> const pt(ix + Random::Uniform(-0.4, 0.4),
																		 iy + Random::Uniform(-0.4, 0.4));
							glVertex2d(pt.v0, pt.v1);
							pts.push_back(pt);
							glVertex2d(ii->v0 + Random::Uniform(-0.4, 0.4),
												 ii->v1 + Random::Uniform(-0.4, 0.4));
						}
					}
			}
			else {
				const Mapper2d::refmap_t & refmap(m_mapper->GetRefmap());
				glBegin(GL_LINES);
				for (ssize_t ix(xbegin); ix < xend; ++ix)
					for (ssize_t iy(ybegin); iy < yend; ++iy) {
						int val;
						if ( ! rdtravmap->GetValue(ix, iy, val))
							continue;
						const Mapper2d::ref_s & ref(refmap.at(ix, iy));
						for (Mapper2d::rev_t::const_iterator ii(ref.reverse.begin());
								 ii != ref.reverse.end(); ++ii) {
							if (ii->first == val)
								glColor3d(1, 0.5, 0);
							else
								glColor3d(0, 0.5, 1);
							glVertex2d(ix + Random::Uniform(-0.4, 0.4),
												 iy + Random::Uniform(-0.4, 0.4));
							vec2d<double> const pt(ii->second.v0 + Random::Uniform(-0.4, 0.4),
																		 ii->second.v1 + Random::Uniform(-0.4, 0.4));
							glVertex2d(pt.v0, pt.v1);
							pts.push_back(pt);
						}
					}
				glEnd();
			}
		
			glMatrixMode(GL_MODELVIEW);
			glPopMatrix();    
		}
		catch (runtime_error ee) {
			errx(EXIT_FAILURE, "exception in MapperRefDrawing::Draw(): %s", ee.what());
		}
  }

}
