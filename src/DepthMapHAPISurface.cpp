//////////////////////////////////////////////////////////////////////////////
//    Copyright 2004, SenseGraphics AB
//
//    This file is part of H3D API.
//
//    H3D API is free software; you can redistribute it and/or modify
//    it under the terms of the GNU General Public License as published by
//    the Free Software Foundation; either version 2 of the License, or
//    (at your option) any later version.
//
//    H3D API is distributed in the hope that it will be useful,
//    but WITHOUT ANY WARRANTY; without even the implied warranty of
//    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//    GNU General Public License for more details.
//
//    You should have received a copy of the GNU General Public License
//    along with H3D API; if not, write to the Free Software
//    Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
//
//    A commercial license is also available. Please contact us at 
//    www.sensegraphics.com for more information.
//
//
/// \file DepthMapHAPISurface.cpp
/// \brief cpp file for DepthMapHAPISurface
///
//
//////////////////////////////////////////////////////////////////////////////
#include "DepthMapHAPISurface.h"

using namespace HAPI;

DepthMapHAPISurface::DepthMapHAPISurface( HAPIFloat _stiffness,
                                  HAPIFloat _static_friction,
                                  HAPIFloat _dynamic_friction,
                                  H3DUtil::Image * _depth_map,
                                  HAPIFloat _depth_map_scale,
                                  bool _white_max,
                                  int _max_iterations,
                                  HAPIFloat _minimization_epsilon ) :
  HAPIVariableDepthSurface( _stiffness, _static_friction, _dynamic_friction,
                         scaleDepth, _max_iterations, _minimization_epsilon ),
  ImageInterfaceObject( _depth_map ),
  depth_map_scale( _depth_map_scale ),
  white_max( _white_max )
{
  image_lock = &depth_get_lock;
}

HAPIFloat DepthMapHAPISurface::getDepthMapValue( Vec3 tex_coord ) {

  // still need to normalize the texture coordinates in a better way.
  if( tex_coord.x < 0.0 )
    tex_coord.x = 0.0;
  if( tex_coord.x > 1.0 )
    tex_coord.x = 1.0;

  if( tex_coord.y < 0.0 )
    tex_coord.y = 0.0;
  if( tex_coord.y > 1.0 )
    tex_coord.y = 1.0;

  if( tex_coord.z < 0.0 )
    tex_coord.z = 0.0;
  if( tex_coord.z > 1.0 )
    tex_coord.z = 1.0;

  return getSample( tex_coord );
}
