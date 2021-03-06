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

#include "ObjectCamera.hpp"
#include "View.hpp"


namespace npm2 {
  

  ObjectCamera::
  ObjectCamera (string const & name)
    : Camera (name, "bounds of the tree rooted at a given object"),
      object_ (0),
      margin_ (0.1)
  {
    reflectSlot ("object", &object_);
    reflectParameter ("margin", &margin_);
  }
  
  
  void ObjectCamera::
  configureView (View & view)
  {
    BBox const & bbox (object_->getBBox());
    if (bbox.isValid()) {
      view.setBounds (bbox, margin_);
    }
    else {
      view.setBounds (0.0, 0.0, 1.0, 1.0);
    }
  }
  
}
