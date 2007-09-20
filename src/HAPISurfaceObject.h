//////////////////////////////////////////////////////////////////////////////
//    Copyright 2004-2007, SenseGraphics AB
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
#include <RefCountedClass.h>

namespace HAPI {

  /// The HAPISurfaceObject is the base class of all surfaces in HAPI.
  /// A surface is responsible for defining the forces and proxy movements
  /// to use when in contact with a shape, depending on penetration depth,
  /// friction etc. Surfaces that are to work with RuspiniRenderer and
  /// GodObjectRenderer has to override the virtual functions getForces
  /// and getProxyMovement
  class HAPI_API HAPISurfaceObject:public H3DUtil::RefCountedClass {
  public:

    /// Structure that contains information about a contact.
    struct HAPI_API ContactInfo {
      ContactInfo(): has_inverse( false ),
                     proxy_radius( 0 ) {}
    protected:
      // INPUT

      // The point of contact in global coordinates.
      Vec3 contact_point_global;

      // The position of the haptics device in global coordinates.
      Vec3 probe_position_global;

      // The texture coordinate at the contact point.
      Vec3 tex_coord;

      // The axes of the local coordinate space in global coordinates.
      Vec3 x_axis;
      Vec3 y_axis;
      Vec3 z_axis;

      // The origin of the local coordinate space in global coordinates.
      Vec3 origin_global;
      
      // OUTPUT

      // output force in global coordinates.
      Vec3 force_global;

      // output torque in global coordinates.
      Vec3 torque_global;

      Matrix3 force_position_jacobian;
      Matrix3 force_velocity_jacobian;

      // The proxy movement in local coordinates.
      Vec2 proxy_movement_local;

      // The proxy radius(if applicable, -1 if the proxy radius does 
      // not make sense for the used haptics renderer)
      HAPIFloat proxy_radius;

      // The id of the shape of the contact.
      int shape_id;

      // local

      // true if the inverse member has been calculated
      // (from global to local space)
      bool has_inverse;

      // The matrix from global to local space (if the has_inverse member
      // is true, otherwise a call to updateInverse is needed)
      Matrix4 inverse;

      // The haptics device that has made the contact.
      // HAPIHapticsDevice *hd;

      // Set the origin of the local coordinate system.
      inline void setGlobalOrigin( const Vec3 &o ) {
        origin_global = o;
        has_inverse = false;
      }

      // Update the inverse member.
      inline void updateInverse() {
        inverse = 
          Matrix4( x_axis.x, y_axis.x, z_axis.x, origin_global.x,
                   x_axis.y, y_axis.y, z_axis.y, origin_global.y,
                   x_axis.z, y_axis.z, z_axis.z, origin_global.z,
                   0,0,0,1 ).transformInverse();
        has_inverse = true;
      }

    public:

      /// Transform a point from local to global space.
      inline Vec3 pointToGlobal( const Vec3 &p ) { 
        return p.x * x_axis + p.y * y_axis + p.z * z_axis + 
          origin_global;
      }
      
      /// Transform a point from global to local space.
      inline Vec3 pointToLocal( const Vec3 &p ) { 
        if( !has_inverse ) updateInverse();
        return inverse * p; 
      }

      /// Transform a vector from local to global space.
      inline Vec3 vectorToGlobal( const Vec3 &p ) { 
        return p.x * x_axis + p.y * y_axis + p.z * z_axis;
      }

      /// Transform a vector from global to local space.
      inline Vec3 vectorToLocal( const Vec3 &p ) {
        //if( !has_inverse ) updateInverse();
        return Matrix3( x_axis.x, y_axis.x, z_axis.x,
                        x_axis.y, y_axis.y, z_axis.y,
                        x_axis.z, y_axis.z, z_axis.z ).inverse() * p;
      }
      
      /// Get the contact point in global coordinates(if using RuspiniRenderer
      /// this will be a point proxy_radius above the actual shape surface).
      inline Vec3 globalContactPoint() {
        return contact_point_global;
      }

      /// Get the contact point in local coordinates(if using RuspiniRenderer
      /// this will be a point proxy_radius above the actual shape surface).
      inline Vec3 localContactPoint() {
        return pointToLocal( contact_point_global );
      }

      /// Get the contact point in global coordinates. This point is always 
      /// on the surface of the shape(even when using RuspiniRenderer)
      inline Vec3 globalSurfaceContactPoint() {
        return contact_point_global - proxy_radius * y_axis;
      }

      /// Get the contact point in local coordinates. This point is always 
      /// on the surface of the shape(even when using RuspiniRenderer)
      inline Vec3 localSurfaceContactPoint() {
        return pointToLocal( globalSurfaceContactPoint() );
      }

      /// Get the probe position in global coordinates.
      inline Vec3 globalProbePosition() {
        return probe_position_global;
      }

      /// Get the probe position in local coordinates.
      inline Vec3 localProbePosition() {
        return pointToLocal( probe_position_global );
      }

      /// Get the origin of the local coordinate system in global coordinates.
      inline Vec3 globalOrigin() {
        return origin_global;
      }

      /// Set the output force in local coordinates. 
      inline void setLocalForce( const Vec3 &f ) {
        force_global = vectorToGlobal( f );
      }

      /// Set the output force in global coordinates.
      inline void setGlobalForce( const Vec3 &f ) {
        force_global = f;
      }

      /// Set the output torque in local coordinates. 
      inline void setLocalTorque( const Vec3 &t ) {
        torque_global = vectorToGlobal( t );
      }

      /// Set the output torque in glocal coordinates. 
      inline void setGlobalTorque( const Vec3 &t ) {
        torque_global = t;
      }
      
      inline void setLocalProxyMovement( const Vec2 &pm ) {
        proxy_movement_local = pm;
      }

      friend class RuspiniRenderer;
      friend class GodObjectRenderer;
    }; 

    /// Destructor.
    virtual ~HAPISurfaceObject();

    /// This function should be overridden by all subclasses. It determines
    /// the proxy movement of a contact. The function should set the proxy
    /// movement in the contact_info struct before returning. During the 
    /// call to this function the origin of the local coordinate system is
    /// the proxy position/contact point.
    virtual void getProxyMovement( ContactInfo &contact_info ) {}

    /// This function should be overridden by all subclasses. It determines
    /// the forces generated at a contact. During the call to this function
    /// the origin of the local coordinate system is the proxy position
    /// after proxy movement.
    virtual void getForces( ContactInfo &contact_info ) {}

  };
}

#endif
