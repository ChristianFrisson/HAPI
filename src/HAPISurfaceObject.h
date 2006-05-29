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
/// \file HAPISurfaceObject.h
/// \brief Header file for HAPISurfaceObject
///
//
//////////////////////////////////////////////////////////////////////////////
#ifndef __HAPISURFACEOBJECT_H__
#define __HAPISURFACEOBJECT_H__

#include <HAPI.h>
#include <H3DTypes.h>

namespace H3D {

  class HAPI_API HAPISurfaceObject {
  public:
    struct HAPI_API SurfaceInput {
      Vec3d contact_point_global;
      Vec3d contact_normal_global;
      Vec3d probe_position_local;
      Vec3d probe_position_global;
      Vec3d tex_coord;
      Vec3d x_axis;
      Vec3d y_axis;
      Vec3d z_axis;
      // HAPIHapticsDevice *hd;

      Vec3d inline pointToGlobal( const Vec3d &p ) { return Vec3d(); }
      Vec3d inline pointToLocal( const Vec3d &p ) { return Vec3d(); }
      Vec3d inline vectorToGlobal( const Vec3d &p ) { return Vec3d(); }
      Vec3d inline vectorToLocal( const Vec3d &p ) { return Vec3d(); }

    }; 

    struct HAPI_API SurfaceOutput {
      Vec3d force;
      Vec3d torque;
      Vec2d proxy_movement;
      Matrix4f force_position_jacobian;
      Matrix4f force_velocity_jacobian;
    }; 

    virtual ~HAPISurfaceObject();

    virtual void onContact( const SurfaceInput &input, SurfaceOutput &output ) {
      
    }
    
    
  };
}

#endif
