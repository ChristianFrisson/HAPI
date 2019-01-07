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
/// \file HAPI/src/DepthMapSurface.cpp
/// \brief cpp file for DepthMapSurface
///
//
//////////////////////////////////////////////////////////////////////////////
#include <HAPI/DepthMapSurface.h>

using namespace HAPI;

DepthMapSurface::DepthMapSurface( HAPIFloat _stiffness,
                                  HAPIFloat _damping,
                                  HAPIFloat _static_friction,
                                  HAPIFloat _dynamic_friction,
                                  H3DUtil::Image * _depth_map,
                                  HAPIFloat _max_depth,
                                  bool _white_max,
                                  bool _use_relative_values,
                                  int _max_iterations,
                                  HAPIFloat _minimization_epsilon,
                                  bool _use_ref_count_lock,
                                  bool _wrap_s,
                                  bool _wrap_t,
                                  bool _wrap_r ) :
  HAPIVariableDepthSurface( _stiffness, _damping,
                            _static_friction, _dynamic_friction,
                            scaleDepth, _max_iterations,
                            _minimization_epsilon, _use_relative_values,
                            _use_ref_count_lock ),
  ImageInterfaceObject( _depth_map ),
  max_depth( _max_depth ),
  white_max( _white_max ),
  wrap_s( _wrap_s ),
  wrap_t( _wrap_t ),
  wrap_r( _wrap_r ) {
  image_lock = &depth_get_lock;
}

HAPIFloat DepthMapSurface::getDepthMapValue( Vec3 tex_coord ) {

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
