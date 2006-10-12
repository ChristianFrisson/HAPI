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
#include <HAPITypes.h>

namespace HAPI {

  class HAPI_API HAPISurfaceObject {
  public:
    struct HAPI_API ContactInfo {
      ContactInfo(): has_inverse( false ) {}

      // input
      Vec3 contact_point_global;
      Vec3 probe_position_global;
      Vec3 tex_coord;
      Vec3 x_axis;
      Vec3 y_axis;
      Vec3 z_axis;


      // output
      Vec3 force_global;
      Vec3 torque_global;
      Matrix4 force_position_jacobian;
      Matrix4 force_velocity_jacobian;

      Vec2 proxy_movement_local;

      HAPIFloat proxy_radius;

      // local
      bool has_inverse;
      Matrix4 inverse;

      // HAPIHapticsDevice *hd;

      inline void updateInverse() {
        inverse = 
          Matrix4( x_axis.x, y_axis.x, z_axis.x, contact_point_global.x,
                   x_axis.y, y_axis.y, z_axis.y, contact_point_global.y,
                   x_axis.z, y_axis.z, z_axis.z, contact_point_global.z,
                   0,0,0,1 ).transformInverse();
        has_inverse = true;
      }

      inline Vec3 pointToGlobal( const Vec3 &p ) { 
        return p.x * x_axis + p.y * y_axis + p.z * z_axis + 
          contact_point_global;
      }
      
      inline Vec3 pointToLocal( const Vec3 &p ) { 
        if( !has_inverse ) updateInverse();
        return inverse * p; 
      }

      inline Vec3 vectorToGlobal( const Vec3 &p ) { 
        return p.x * x_axis + p.y * y_axis + p.z * z_axis;
      }

      inline Vec3 vectorToLocal( const Vec3 &p ) {
        //if( !has_inverse ) updateInverse();
        return Matrix3( x_axis.x, y_axis.x, z_axis.x,
                         x_axis.y, y_axis.y, z_axis.y,
                         x_axis.z, y_axis.z, z_axis.z ).inverse() * p;
      }
      
      inline Vec3 globalContactPoint() {
        return contact_point_global;
      }

      inline Vec3 localContactPoint() {
        return pointToLocal( contact_point_global );
      }

      inline Vec3 globalSurfaceContactPoint() {
        return contact_point_global - proxy_radius * y_axis;
      }

      inline Vec3 localSurfaceContactPoint() {
        return pointToLocal( globalSurfaceContactPoint() );
      }

      inline Vec3 globalProbePosition() {
        return probe_position_global;
      }

      inline Vec3 localProbePosition() {
        return pointToLocal( probe_position_global );
      }

      inline void setLocalForce( const Vec3 &f ) {
        force_global = vectorToGlobal( f );
      }

      inline void setGlobalForce( const Vec3 &f ) {
        force_global = f;
      }
      

    }; 

    struct HAPI_API SurfaceOutput {
      
    }; 

    virtual ~HAPISurfaceObject();

    virtual void onContact( ContactInfo &contact_info ) {
      
    }
    
    
  };
}

#endif
