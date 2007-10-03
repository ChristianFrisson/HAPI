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
/// \file DepthMapHAPISurface.h
/// \brief Header file for DepthMapHAPISurface
///
//
//////////////////////////////////////////////////////////////////////////////
#ifndef __DEPTHMAPHAPISURFACE_H__
#define __DEPTHMAPHAPISURFACE_H__

#include <HAPIVariableDepthSurface.h>
#include <ImageInterfaceObject.h>

namespace HAPI {
  /// \class DepthMapHAPISurface
  /// A HAPIVariableDepthSurface using a texture to calculate depth.
  /// For OpenHapticsRenderer this surface will not work.
  class HAPI_API DepthMapHAPISurface: public HAPIVariableDepthSurface,
                                      public ImageInterfaceObject {

  public:
    /// Constructor
    DepthMapHAPISurface( HAPIFloat _stiffness = 0.35,
                         HAPIFloat _static_friction = 0.1,
                         HAPIFloat _dynamic_friction = 0.4,
                         H3DUtil::Image * _depth_map = 0,
                         HAPIFloat _max_depth = 1.0,
                         bool _white_max = true,
                         int _max_iterations = 35,
                         HAPIFloat _minimization_epsilon = 1e-4,
                         bool _use_ref_count_lock = true );

  protected:
    /// Get the value of the texture at texture coordinate tex_coord.
    /// For any image other than a grayscale image the red channel of
    /// the returned RGBA value will be used.
    HAPIFloat getDepthMapValue( Vec3 tex_coord );

    /// Function used as function parameter to HAPIVariableDepthSurface.
    /// \param local_point point in local coordinate of the contact on the
    /// the surface.
    /// \param data contains a pointer to the DepthMapHAPISurface calling this
    /// function.
    static inline HAPIFloat scaleDepth( const Vec2 &local_point, void *data ) {
      DepthMapHAPISurface *dms = static_cast< DepthMapHAPISurface * >(data);
      Matrix4 tangent_space_mtx;
      Vec3 global_vector = dms->this_contact_info->vectorToGlobal(
                            Vec3( local_point.x, 0, local_point.y ) );
      dms->this_contact_info->primitive->getTangentSpaceMatrix(
        dms->this_contact_info->globalOrigin() + global_vector,
        tangent_space_mtx );
      HAPIFloat depth_value = dms->getDepthMapValue(
          dms->this_contact_info->tex_coord +
          tangent_space_mtx *
          global_vector );
      if( dms->white_max )
        return dms->max_depth * ( depth_value - 1 );
      else
        return -dms->max_depth * depth_value;
    }

    /// Scale the range of the pixel values in the texture used.
    /// The value range will be 0 - max_depth
    HAPIFloat  max_depth;

    /// If true then white ( pixel value 1 ) will be considered a high
    /// point in the texture.
    bool white_max;
  };
}

#endif