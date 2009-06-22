/*
 * Copyright (c) 2008 Roland Philippsen <roland DOT philippsen AT gmx DOT net>
 *
 * BSD license:
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holder nor the names of
 *    contributors to this software may be used to endorse or promote
 *    products derived from this software without specific prior written
 *    permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHORS AND CONTRIBUTORS ``AS IS''
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED
 * TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
 * PARTICULAR PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT
 * HOLDER OR THE CONTRIBUTORS TO THIS SOFTWARE BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include "VisualRobox.hpp"
#include "MPDrawing.hpp"
#include "OCamera.hpp"
#include "ODrawing.hpp"
#include "DODrawing.hpp"
#include "DWDrawing.hpp"
#include "RHDrawing.hpp"
#include "BBDrawing.hpp"
#include "GridLayerCamera.hpp"
#include "GridLayerDrawing.hpp"
#include "../common/OdometryDrawing.hpp"
#include "../common/StillCamera.hpp"
#include "../common/Manager.hpp"
#include <sfl/expo/MotionPlanner.hpp>
#include <sfl/bband/ReplanHandler.hpp>
#include <sfl/dwa/DynamicWindow.hpp>
#include <sfl/dwa/DistanceObjective.hpp>
#include <sfl/dwa/HeadingObjective.hpp>
#include <sfl/dwa/SpeedObjective.hpp>
#include <sfl/api/RobotModel.hpp>

using namespace boost;


namespace npm {  
  
  VisualRobox::
  VisualRobox(std::string const & name,
	      expo_parameters const & params,
	      boost::shared_ptr<sfl::HAL> hal,
	      boost::shared_ptr<sfl::Multiscanner> mscan,
	      bool use_tobi_distobj)
    : expo::Robox(params, hal, mscan, use_tobi_distobj)
  {
    AddDrawing(new MPDrawing(name + "_goaldrawing", *motionPlanner));
    AddDrawing(new DWDrawing(name + "_dwdrawing", *dynamicWindow));
    AddDrawing(new ODrawing(name + "_dodrawing", distanceObjective, dynamicWindow));
    AddDrawing(new ODrawing(name + "_hodrawing", headingObjective, dynamicWindow));
    AddDrawing(new ODrawing(name + "_sodrawing", speedObjective, dynamicWindow));
    sfl::ReplanHandler const * rph(dynamic_cast<sfl::ReplanHandler const *>(bubbleBand->GetReplanHandler()));
    if (rph) {
      AddDrawing(new RHDrawing(name + "_rhdrawing",
			       rph,
			       RHDrawing::AUTODETECT));
    }
    AddDrawing(new BBDrawing(name + "_bbdrawing",
			     *bubbleBand,
			     BBDrawing::AUTODETECT));
    if (rph) {
      AddDrawing(new GridLayerDrawing(name + "_local_gldrawing",
				      rph->GetNF1(),
				      false));
      AddDrawing(new GridLayerDrawing(name + "_global_gldrawing",
				      rph->GetNF1(),
				      true));
    }
    AddDrawing(new OdometryDrawing(name + "_odomdrawing",
				   *odometry,
				   robotModel->WheelBase() / 2));
    AddDrawing(new DODrawing(name + "_collisiondrawing",
			     distanceObjective,
			     headingObjective,
			     dynamicWindow,
			     robotModel));
    
    AddCamera(new StillCamera(name + "_dwcamera",
			      0,
			      0,
			      dynamicWindow->Dimension(),
			      dynamicWindow->Dimension(),
			      Instance<UniqueManager<Camera> >()));
    AddCamera(new OCamera(name + "_ocamera", *dynamicWindow));
    if (rph) {
      AddCamera(new GridLayerCamera(name + "_local_glcamera",
				    rph->GetNF1()));
    }
    double a, b, c, d;
    distanceObjective->GetRange(a, b, c, d);
    AddCamera(new StillCamera(name + "_collisioncamera", a, b, c, d,
			      Instance<UniqueManager<Camera> >()));
  }
  
  
  void VisualRobox::
  AddDrawing(Drawing * drawing)
  {
    m_drawing.push_back(shared_ptr<Drawing>(drawing));
  }
  
  
  void VisualRobox::
  AddCamera(Camera * camera)
  {
    m_camera.push_back(shared_ptr<Camera>(camera));
  }
  
}
