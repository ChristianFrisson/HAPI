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
/// \file HAPISurfaceObject.h
/// \brief Header file for HAPISurfaceObject
///
//
//////////////////////////////////////////////////////////////////////////////
#ifndef __HAPISURFACEOBJECT_H__
#define __HAPISURFACEOBJECT_H__

#include <HAPI/HAPI.h>
#include <HAPI/HAPITypes.h>
#include <HAPI/CollisionObjects.h>

#include <H3DUtil/RefCountedClass.h>

namespace HAPI {

  class HAPIHapticsDevice;
  class HAPIHapticShape;

  /// \ingroup AbstractClasses
  /// \class HAPISurfaceObject
  /// \brief The base class of all surfaces in HAPI.
  ///
  /// A surface is responsible for defining the forces and proxy movements
  /// to use when in contact with a shape, depending on penetration depth,
  /// friction etc. Surfaces that are to work with RuspiniRenderer and
  /// GodObjectRenderer has to override the virtual functions getForces
  /// and getProxyMovement
  class HAPI_API HAPISurfaceObject:public H3DUtil::RefCountedClass {
  public:

    /// Structure that contains information about a contact.
    struct HAPI_API ContactInfo {
      /// Default constructor.
      ContactInfo(): geom_primitive( 0 ),
                     haptic_shape( 0 ),
                     proxy_radius( 0 ),
                     has_inverse( false ),
                     hd( NULL ),
                     shape_id( 0 ) {}
    protected:
      // INPUT

      // The point of contact in global coordinates.
      Vec3 contact_point_global;

      // The position of the haptics device in global coordinates.
      Vec3 probe_position_global;

      // The velocity of the haptics device in global coordinates.
      Vec3 probe_velocity_global;

      // The texture coordinate at the contact point.
      Vec3 tex_coord;

      // The axes of the local coordinate space in global coordinates.
      Vec3 x_axis;
      Vec3 y_axis;
      Vec3 z_axis;

      // The origin of the local coordinate space in global coordinates.
      Vec3 origin_global;

      // Contains the primitive on which the point of contact was detected.
      Collision::GeometryPrimitive * geom_primitive;

      // Contains a pointer to the HAPIHapticShape on which the point of
      // contact was detected
      HAPIHapticShape *haptic_shape;
      
      // OUTPUT

      // output force in global coordinates.
      Vec3 force_global;

      // output torque in global coordinates.
      Vec3 torque_global;

      Matrix3 force_position_jacobian;
      Matrix3 force_velocity_jacobian;

      // The proxy movement in local coordinates.
      Vec2 proxy_movement_local;

      ///////////////////////////////////////////////////

      // The proxy radius(if applicable, 0 if the proxy radius does 
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
      HAPIHapticsDevice *hd;

      // Update the inverse member.
      inline void updateInverse() {
        inverse = localToGlobalMatrix().transformInverse();
        has_inverse = true;
      }

    public:

      /// Get the force currently assigned to the contact.
      inline Vec3 globalForce() {
        return force_global;
      }

      /// Get the torque currently assigned to the contact.
      inline Vec3 globalTorque() {
        return torque_global;
      }

      /// Get the transformation matrix from local surface space to global
      /// space.
      inline Matrix4 localToGlobalMatrix() {
        return  Matrix4( x_axis.x, y_axis.x, z_axis.x, origin_global.x,
                         x_axis.y, y_axis.y, z_axis.y, origin_global.y,
                         x_axis.z, y_axis.z, z_axis.z, origin_global.z,
                         0,0,0,1 );
      }

      /// Get the transformation matrix from global surface space to local
      /// space.
      inline Matrix4 globalToLocalMatrix() {
        if( !has_inverse ) updateInverse();
        return  inverse;
      }

      /// Get the texture coordinate of the contact point.
      inline Vec3 contactPointTexCoord() {
        return tex_coord;
      }

      /// Returns the haptics device that is in contact.
      inline HAPIHapticsDevice *hapticsDevice() {
        return hd;
      }

      /// Returns the GeometryPrimitive that the contact occurs on.
      inline Collision::GeometryPrimitive *primitive() {
        return geom_primitive;
      }

      /// Returns the HAPIHapticShape that the contact occurs on.
      inline HAPIHapticShape *hapticShape() {
        return haptic_shape;
      }

      /// Returns the x-axis of the local coordinate system in global
      /// coordinates.
      inline Vec3 xAxis() {
        return x_axis;
      }

      /// Returns the y-axis of the local coordinate system in global
      /// coordinates.
      inline Vec3 yAxis() {
        return y_axis;
      }

      /// Returns the z-axis of the local coordinate system in global
      /// coordinates.
      inline Vec3 zAxis() {
        return z_axis;
      }

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

      /// Get the probe velocity in global coordinates.
      inline Vec3 globalProbeVelocity() {
        return probe_velocity_global;
      }

      /// Get the probe velocity in local coordinates.
      inline Vec3 localProbeVelocity() {
        return vectorToLocal( probe_velocity_global );
      }

      /// Get the origin of the local coordinate system in global coordinates.
      inline Vec3 globalOrigin() {
        return origin_global;
      }

      /// Get the proxy position in global coordinates.
      inline Vec3 globalProxyPosition() {
        return origin_global;
      }

      /// Get the local proxy movement set by getProxyMovement function.
      inline Vec2 localProxyMovement() {
        return proxy_movement_local;
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

      /// Set the output torque in global coordinates.
      inline void setGlobalTorque( const Vec3 &t ) {
        torque_global = t;
      }
      
      /// Set the proxy movement in local surface space.
      inline void setLocalProxyMovement( const Vec2 &pm ) {
        proxy_movement_local = pm;
      }

      /// Set the origin of the local coordinate system.
      /// This function should only be called by renderers.
      inline void setGlobalOrigin( const Vec3 &o ) {
        origin_global = o;
        has_inverse = false;
      }

      /// Set the contact_point_global of ContactInfo
      /// This function should only be called by renderers.
      inline void setGlobalContactPoint( const Vec3 &point ) {
        contact_point_global = point;
      }

      /// Set the x_axis of ContactInfo
      /// This function should only be called by renderers.
      inline void setXAxis( const Vec3 &axis ) {
        x_axis = axis;
      }

      /// Set the y_axis of ContactInfo
      /// This function should only be called by renderers.
      inline void setYAxis( const Vec3 &axis ) {
        y_axis = axis;
      }

      /// Set the z_axis of ContactInfo
      /// This function should only be called by renderers.
      inline void setZAxis( const Vec3 &axis ) {
        z_axis = axis;
      }

      /// Set the haptic_shape member of ContactInfo
      /// This function should only be called by renderers.
      inline void setHapticShape( HAPIHapticShape * shape ) {
        haptic_shape = shape;
      }

      /// Set the geom_primitive member of ContactInfo
      /// This function should only be called by renderers.
      inline void setPrimitive( Collision::GeometryPrimitive * primitive ) {
        geom_primitive = primitive;
      }

      /// Set the hd member of ContactInfo
      /// This function should only be called by renderers.
      inline void setHapticsDevice( HAPIHapticsDevice *_hd ) {
        hd = _hd;
      }

      /// Set the probe_position_global member of ContactInfo.
      /// This function should only be called by renderers.
      inline void setGlobalProbePosition( const Vec3 &point ) {
        probe_position_global = point;
      }

      /// Set the probe_velocity_global member of ContactInfo.
      /// This function should only be called by renderers.
      inline void setGlobalProbeVelocity( const Vec3 &velocity ) {
        probe_velocity_global = velocity;
      }

      /// Set the proxy_radius member of ContactInfo.
      /// This function should only be called by renderers.
      inline void setProxyRadius( HAPIFloat radius ) {
        proxy_radius = radius;
      }

      /// Set the tex_coord member of ContactInfo.
      /// This function should only be called by renderers.
      inline void setContactPointTexCoord( const Vec3 &coord ) {
        tex_coord = coord;
      }

      friend class RuspiniRenderer;
      friend class GodObjectRenderer;
      friend class OpenHapticsRenderer;
      friend class Chai3DRenderer;
    }; 

    /// Constructor
    HAPISurfaceObject( bool _use_ref_count_lock = true );

    /// Destructor.
    virtual ~HAPISurfaceObject();

    /// This function should be overridden by all subclasses. It determines
    /// the proxy movement of a contact. The function should set the proxy
    /// movement in the contact_info struct before returning using the
    /// setLocalProxyMovement function. During the 
    /// call to this function the origin of the local coordinate system is
    /// the proxy position/contact point.
    /// \param contact_info Contains information about the contact. The
    /// function must set the proxy movement by calling setLocalProxyMovement
    /// function.
    virtual void getProxyMovement( ContactInfo &contact_info ) {}

    /// This function should be overridden by all subclasses. It determines
    /// the forces generated at a contact by using the setLocalForce or
    /// setGlobalForce functions in the ContactInfo struct. 
    /// During the call to this function the origin of the local coordinate
    /// system is the proxy position after proxy movement.
    virtual void getForces( ContactInfo &contact_info ) {}

  };
}

#endif
