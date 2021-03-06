/* Nepumuk Mobile Robot Simulator v2
 *
 * Copyright (C) 2014 Roland Philippsen. All rights reserved.
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

#include "Body.hpp"

namespace npm2 {
  
  
  void Body::
  addLine (Line const & line)
  {
    local_lines_.push_back (line);
  }
  
  
  void Body::
  addLine (double x0, double y0, double x1, double y1)
  {
    local_lines_.push_back (Line (x0, y0, x1, y1));
  }

  
  void Body::
  transformTo (Frame const & global)
  {
    if (global_lines_.size() != local_lines_.size()) {
      global_lines_.resize (local_lines_.size());
    }
    
    bbox_.reset();
    for (size_t il(0); il < local_lines_.size(); ++il) {
      global_lines_[il] = local_lines_[il];
      global_lines_[il].TransformTo (global);
      bbox_.update (global_lines_[il]);
    }
  }
  
}
