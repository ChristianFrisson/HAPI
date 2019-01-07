//////////////////////////////////////////////////////////////////////////////
//    Copyright 2004-2019, SenseGraphics AB
//
//    This file is part of HAPI.
//
//    HAPI is free software; you can redistribute it and/or modify
//    it under the terms of the GNU General Public License as published by
//    the Free Software Foundation; either version 2 of the License, or
//    (at your option) any later version.
//
//    HAPI is distributed in the hope that it will be useful,
//    but WITHOUT ANY WARRANTY; without even the implied warranty of
//    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//    GNU General Public License for more details.
//
//    You should have received a copy of the GNU General Public License
//    along with HAPI; if not, write to the Free Software
//    Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
//
//    A commercial license is also available. Please contact us at 
//    www.sensegraphics.com for more information.
//
//
/// \file HLCustomObject.cpp
/// \brief cpp file for HLCustomObject
///
//
//////////////////////////////////////////////////////////////////////////////


#include <HAPI/HLCustomObject.h>
#include <HAPI/HAPIHapticShape.h>
#include <HAPI/OpenHapticsRenderer.h>

#ifdef HAVE_OPENHAPTICS

using namespace HAPI;

void HLCustomObject::hlRender( HAPIHapticsDevice *hd,
                               HLuint hl_shape_id ) {
  if( OpenHapticsRenderer::hlRenderHAPISurface( surface.get(), hd ) ) {
    hlBeginShape(HL_SHAPE_CALLBACK, hl_shape_id );
    hlCallback(HL_SHAPE_INTERSECT_LS, 
               (HLcallbackProc) intersectCallback, this);
    hlCallback(HL_SHAPE_CLOSEST_FEATURES, 
               (HLcallbackProc) closestFeaturesCallback, this);
    hlEndShape();
  }
};
#endif
