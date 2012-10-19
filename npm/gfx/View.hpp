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


#ifndef NPM_VIEW_HPP
#define NPM_VIEW_HPP


#include <npm/Manageable.hpp>
#include <vector>


namespace npm {
  
  class Camera;
  class BBox;
  class Drawing;
  
  /**
     \brief Subwindow.
     
     A View is a subwindow of the graphic output. It has grown out of
     the need to handle OpenGL projection matrix and viewport
     configuration.
     
     The ViewManager (singleton) keeps a list of views that can then be
     requested by the Simulator. Each View defines a region of graphic
     output, and maintains a list of Drawing instances that should be
     drawn into that region. An associated Camera is used to set the
     bounding box of the View's contents. This bounding box is
     independent of the View's location in the main graphics window
     (which is defined during construction or using View::Redefine() or
     View::Configure()).
  */
  class View
    : public Manageable
  {
  public:
    typedef enum { N, NE, E, SE, S, SW, W, NW, CENTER } anchor_t;
    
    /** View instances always need a Manager. */
    View(const std::string & name, boost::shared_ptr<Manager> manager);
    
    void Configure(/// x-coordinate of lower-left corner, range = 0 to 1
		   double x,
		   /// y-coordinate of lower-left corner, range = 0 to 1
		   double y,
		   /// width (along y-axis), range = 0 to 1
		   double width,
		   /// height (along x-axis), range = 0 to 1
		   double height,
		   /// number of pixels to be subtracted around the edges
		   int border = 0,
		   /// how to anchor the drawn scene
		   anchor_t anchor = CENTER,
		   /// use an aspect ratio of 1, or scale to fit along x and y
		   bool lock_aspect = true);
    
    bool SetCamera(const std::string & name);
    bool AddDrawing(const std::string & name);
    void SavePNG(const std::string & filename);
    void SavePNG();
    
    void Draw();
    
    struct DrawWalker {
      DrawWalker() {}
      void Walk(View * host) { host->Draw(); }
    };

    void Redefine(double x, double y, double width, double height);
    void SetBorder(int border);
    void SetAnchor(anchor_t anchor);
    
    /// Set the bounding box of what's to be drawn inside the View.
    void SetBounds(const BBox & bbox, double margin = 0);
    
    /// Set the bounding box of what's to be drawn inside the View.
    void SetBounds(double x0, double y0, double x1, double y1,
		   double margin = 0);
    
    /// Deprecated in favor of View::SetBounds() because the order of
    /// parameters is a bit bizarre
    void SetRange(double x0, double x1, double y0, double y1);
    
    /// Set the modelview transformation to be applied before any
    /// registered drawings are activated. This maps the given (x, y) to
    /// the view's origin, with the view's x-axis aligned along theta.
    /// \note The default modelview is (0, 0, 0), which corresponds to
    /// the identity matrix and thus doesn't attempt to modify the
    /// coordinate frame underlying the drawing. Use UnsetModelview() to
    /// revert to that default behavior.
    /// \todo THIS WAS NEVER TESTED
    void SetModelview(double x, double y, double theta);
    
    /// Reset the modelview transformation to identity.
    /// \todo THIS WAS NEVER TESTED
    void UnsetModelview();
    
    /// Inform the View of a reshape event of the main graphics window.
    void Reshape(int width,	///< new window width (in pixels)
		 int height	///< new window height (in pixels)
		 );
    
    struct ReshapeWalker {
      ReshapeWalker(int _width, int _height)
	: width(_width), height(_height) {}
      void Walk(View * host) { host->Reshape(width, height); }
      int width, height;
    };
      
    /// Prepare the OpenGL parameters such that subsequent drawing
    /// commands draw inside the View, with the bounding box correctly
    /// set.
    void PrepareProjection() const;
    
    double BaseWidth() const;
    double BaseHeight() const;
    double BaseX() const;
    double BaseY() const;
    
    void UnlockAspectRatio();
    void LockAspectRatio();
    
    bool HaveCamera() const;
    
  private:
    Camera * camera;
    std::vector<Drawing *> drawing;
    
    double basewidth, baseheight, basex, basey;
    int totalwidth, totalheight;
    int winwidth, winheight, winx, winy;
    double viewwidth, viewheight, viewx, viewy;
    double xmin, xmax,  ymin,  ymax;
    bool left, right, above, below;
    int winborder, doublewinborder;
    bool lockAspectRatio;
    
    int savecount;
    
    bool mv_enable;
    double mv_x, mv_y, mv_theta_deg;
    
    double Winwidth() const;
    double Winheight() const;
    int    Viewx() const;
    int    Viewy() const;
    int    Viewwidth() const;
    int    Viewheight() const;
    
    void CalculateViewport();
    
    std::string anchor_string;
  };

}

#endif // NPM_VIEW_HPP
