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

#include "Viewport.hpp"
#include "gl.hpp"
#include <limits>
#include <cmath>

namespace npm2 {
  namespace gl {
    
    
    Viewport::
    Viewport ()
    {
      updateSubwin (0.0, 0.0, 1.0, 1.0);
      updateShape (100, 100);
      updateBounds (0.0, 0.0, 1.0, 1.0);
    }
    
    
    Viewport::
    Viewport (double subwin_x0, double subwin_y0, double subwin_x1, double subwin_y1)
    {
      updateSubwin (subwin_x0, subwin_y0, subwin_x1, subwin_y1);
      updateShape (100, 100);
      updateBounds (0.0, 0.0, 1.0, 1.0);
    }
    
    
    void Viewport::
    updateSubwin (double x0, double y0, double x1, double y1)
    {
      subwin_.x0 = x0;
      subwin_.y0 = y0;
      subwin_.width = x1 - x0;
      subwin_.height = y1 - y0;
      dirty_ = true;
    }
    
    
    void Viewport::
    updateShape (int width, int height)
    {
      shape_.winwidth = width;
      shape_.winheight = height;
      dirty_ = true;
    }
    
    
    void Viewport::
    updateBounds (double x0, double y0, double x1, double y1)
    {
      bounds_.x0 = x0 < x1 ? x0 : x1;
      bounds_.y0 = y0 < y1 ? y0 : y1;
      bounds_.x1 = x0 > x1 ? x0 : x1;
      bounds_.y1 = y0 > y1 ? y0 : y1;
      bounds_.cx = (bounds_.x0 + bounds_.x1) / 2.0;
      bounds_.cy = (bounds_.y0 + bounds_.y1) / 2.0;
      dirty_ = true;
    }
    
    
    void Viewport::
    pushOrtho ()
    {
      if (dirty_) {
	updatePadding ();
	dirty_ = false;
      }
      
      glViewport (padding_.x0, padding_.y0, padding_.width, padding_.height);
      gluLookAt (bounds_.cx, bounds_.cy, 1.0, // eye
		 bounds_.cx, bounds_.cy, 0.0, // center
		 0.0, 1.0, 0.0		      // up
		 );
      
      glMatrixMode (GL_PROJECTION);
      glPushMatrix ();
      glLoadIdentity ();
      glOrtho (bounds_.x0, bounds_.x1,
	       bounds_.y0, bounds_.y1,
	       -0.5, 0.5);
    }
    
    
    void Viewport::
    pop ()
    {
      glMatrixMode(GL_PROJECTION);
      glPopMatrix();  
    }
    
    
    void Viewport::
    updatePadding ()
    {
      shape_.x0 = ceil (shape_.winwidth * subwin_.x0);
      shape_.y0 = ceil (shape_.winheight * subwin_.y0);
      shape_.width = floor (shape_.winwidth * subwin_.width);
      shape_.height = floor (shape_.winheight * subwin_.height);
      
      int const pad ((shape_.width - shape_.height) / 2);
      if (pad > 0) {		// pad left and right
	padding_.x0 = shape_.x0 + pad;
	padding_.y0 = shape_.y0;
	padding_.width = shape_.height;
	padding_.height = shape_.height;
      }
      else if (pad < 0) {	// pad below and above
	padding_.x0 = shape_.x0;
	padding_.y0 = shape_.y0 - pad; // notice pad is negative
	padding_.width = shape_.width;
	padding_.height = shape_.width;
      }
      else {
	padding_.x0 = shape_.x0;
	padding_.y0 = shape_.y0;
	padding_.width = shape_.width;
	padding_.height = shape_.height;
      }
    }
  }
  
}
