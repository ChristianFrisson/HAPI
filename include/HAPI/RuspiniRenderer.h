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
/// \file RuspiniRenderer.h
/// \brief Header file for RuspiniRenderer.
///
//
//////////////////////////////////////////////////////////////////////////////

#ifndef __RUSPINIRENDERER_H__
#define __RUSPINIRENDERER_H__

#include <HAPI/HAPIProxyBasedRenderer.h>
#include <HAPI/PlaneConstraint.h>

#include <H3DUtil/Threads.h>
#include <map>

namespace HAPI {

  /// \ingroup Renderers
  /// \class RuspiniRenderer
  /// \brief Haptics renderer that uses a variant of the algorithm presented
  /// in the paper "The haptic display of complex graphical environments"
  /// by Diego C. Ruspini, Krasimir Kolarov and Oussama Khatib.
  ///
  /// This renderer allows for variable proxy radius, i.e. a sphere proxy.
  /// This version of RuspiniRenderer will return torque if the surface given
  /// to the renderer returns torque.
  class HAPI_API RuspiniRenderer: public HAPIProxyBasedRenderer {
  public:
    typedef Collision::CollisionObject::Constraints Constraints;

    /// Constructor.
    RuspiniRenderer( HAPIFloat _proxy_radius = 0.0025,
                     bool _alwaysFollowSurface = false );

    /// Destructor.
    virtual ~RuspiniRenderer() {}

    /// The main function in any haptics renderer. Given a haptics device and 
    /// a group of shapes generate the force and torque to send to the device.
    virtual HAPIForceEffect::EffectOutput
      renderHapticsOneStep(  HAPIHapticsDevice *hd,
                             const HapticShapeVector &shapes,
                             HAPITime dt );

    /// Get the current position of the proxy.
    inline virtual Vec3 getProxyPosition() {
      Vec3 tmp_proxy_pos;
      proxy_info_lock.lock();
      tmp_proxy_pos = proxy_position;
      proxy_info_lock.unlock();
      return tmp_proxy_pos;
    }

    /// Get the current radius of the proxy.
    inline HAPIFloat getProxyRadius() {
      HAPIFloat tmp_proxy_radius;
      proxy_info_lock.lock();
      tmp_proxy_radius = proxy_radius;
      proxy_info_lock.unlock();
      return tmp_proxy_radius;
    }

    /// Set the radius of the proxy.
    inline void setProxyRadius( HAPIFloat r ) {
      proxy_info_lock.lock();
      proxy_radius = r;
      proxy_info_lock.unlock();
    }
    
  /// Always move the proxy when the object that it is in contact with moves.
  ///
  /// Usually the proxy is only moved with the object if the object is 
  /// moving towards the proxy.
  ///
  /// Enabling this option stops the proxy from slipping on the surface when 
  /// the surface is moving tangentially. This helps when you want to drag
  /// an object using the surface friction between the surface and the 
  /// proxy.
  ///
  /// However, enabling this option has the side-effect that the proxy
  /// may fall through a surface where objects intersect.
  ///
  inline void setAlwaysFollowSurface( bool a ) {
    alwaysFollowSurface = a;
  }

    /// Register this renderer to the haptics renderer database.
    static HapticsRendererRegistration renderer_registration;

  protected:
    // If the proxy is in contact with only one single plane this function
    // will take care of force calculations and proxy movement.
    void onOnePlaneContact( const Vec3& proxy_pos,
                            const PlaneConstraint &c, 
                            HAPISurfaceObject::ContactInfo &contact );

    // If the proxy is in contact with two planes this function
    // will take care of force calculations and proxy movement.
    // Might call onOnePlaneContact after further reducing the problem of
    // how many planes the proxy is in contact with.
    void onTwoPlaneContact( const Vec3& proxy_pos,
                            const PlaneConstraint &p0,
                            const PlaneConstraint &p1,
                            HAPISurfaceObject::ContactInfo &contact );

    // If the proxy is in contact with three or more planes this function
    // will take care of force calculations and proxy movement.
    // Might call onTwoPlaneContact after further reducing the problem of
    // how many planes the proxy is in contact with.
    void onThreeOrMorePlaneContact(  const Vec3& proxy_pos,
                                     Constraints &constraints,
                                     HAPISurfaceObject::ContactInfo &contact );

    // Try to move the proxy to a new location. Will stop at the first
    // intersection with a plane in case of any.
    Vec3 tryProxyMovement( const Vec3 &from,
                           const Vec3 &to,
                           const Vec3 &normal );
    
    HAPIFloat proxy_radius;
    bool alwaysFollowSurface;
    Vec3 proxy_position;
    Contacts tmp_contacts;

    Constraints constraints;
    Constraints closest_constraints;
    Constraints other_constraints;

    // vector with pairs shape_id, transform matrix. It contains the id
    // of the shapes that were in contact during last haptics loop and
    // their transform matrices at the time.
    std::vector< std::pair< int, Matrix4 > > last_contact_transforms;
    H3DUtil::MutexLock proxy_info_lock;
    
  };
}

#endif
