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
/// \file GodObjectRenderer.h
/// \brief Header file for GodObjectRenderer.
///
//
//////////////////////////////////////////////////////////////////////////////

#ifndef __GODOBJECTRENDERER_H__
#define __GODOBJECTRENDERER_H__

#include <HAPI/HAPIProxyBasedRenderer.h>
#include <H3DUtil/Threads.h>
#include <HAPI/PlaneConstraint.h>
#include <map>

namespace HAPI {

  /// \ingroup Renderers
  /// \class GodObjectRenderer
  /// \brief Haptics renderer that uses a variant of the algorithm presented
  /// in the paper "A Constraint-based God-object Method For Haptic Display"
  /// by C. B. Zilles J. K. Salisbury.
  ///
  /// The god object is a point that stays on the geometry's surface when the
  /// probe penetrates the surface(which in our implementation is called 
  /// a proxy). This renderer only allows for point proxies.  
  class HAPI_API GodObjectRenderer: public HAPIProxyBasedRenderer {
  public:

    /// Constructor
    GodObjectRenderer(HAPIFloat _min_distance = 1e-7, bool _secondary_collisions = false);

    /// Destructor.
    virtual ~GodObjectRenderer() {}

    /// The main function in any haptics renderer. Given a haptics device and 
    /// a group of shapes, it generates the force and torque to send to the
    /// device. dt is the timestep for the last haptics loop.
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

    inline HAPIFloat getMinDistance() {
      HAPIFloat tmp_min_distance;
      proxy_info_lock.lock();
      tmp_min_distance = min_distance;
      proxy_info_lock.unlock();
      return tmp_min_distance;
    }

    /// Set the minimum distance between the proxy and the surface
    inline void setMinDistance( HAPIFloat m ) {
      proxy_info_lock.lock();
      min_distance = m;
      proxy_info_lock.unlock();
    }

    /// Attempt to ensure that the from point is not placed behind
    /// another surface when calculating proxy movement
    inline void setSecondaryCollisions(bool b) {
      secondary_collisions = b;
    }

    /// Register this renderer to the haptics renderer database.
    static HapticsRendererRegistration renderer_registration;

  protected:
    // If the proxy is in contact with only one single plane this function
    // will take care of force calculations and proxy movement.
    void onOnePlaneContact( const Vec3& proxy_pos,
                            const PlaneConstraint &c, 
                            HAPISurfaceObject::ContactInfo &contact,
                            const HapticShapeVector &shapes );

    // If the proxy is in contact with two planes this function
    // will take care of force calculations and proxy movement.
    // Might call onOnePlaneContact after further reducing the problem of
    // how many planes the proxy is in contact with.
    void onTwoPlaneContact( const Vec3& proxy_pos,
                            const PlaneConstraint &p0,
                            const PlaneConstraint &p1,
                            HAPISurfaceObject::ContactInfo &contact,
                            const HapticShapeVector &shapes );

    // If the proxy is in contact with three or more planes this function
    // will take care of force calculations and proxy movement.
    // Might call onTwoPlaneContact after further reducing the problem of
    // how many planes the proxy is in contact with.
    void onThreeOrMorePlaneContact(  const Vec3& proxy_pos,
                                     std::vector< PlaneConstraint > &constraints,
                                     HAPISurfaceObject::ContactInfo &contact,
                                     const HapticShapeVector &shapes );

    // Try to move the proxy to a new location. Will stop at the first
    // intersection with a plane in case of any.
    bool tryProxyMovement( Vec3 from, Vec3 to, 
                           int nr_constraints,
                           const PlaneConstraint &pc,
                           const HapticShapeVector &shapes,
                           Vec3 &point );

    // The position of the proxy.
    Vec3 proxy_position;

    // The minimum distance between the proxy and the surface
    HAPIFloat min_distance;

    // Attempt to ensure that the from point is not placed behind
    // another surface when calculating proxy movement
    bool secondary_collisions;

    // Tmp container to store contacts in while rendering a new loop.
    Contacts tmp_contacts;
    
    // vector with pairs shape_id, transform matrix. It contains the id
    // of the shapes that were in contact during last haptics loop and
    // their transform matrices at the time.
    std::vector< std::pair< int, Matrix4 > > last_contact_transforms;

    // the intersections that are closest to the user (several if the 
    // intersections are in the same point)
    std::vector< PlaneConstraint > closest_constraints;

    // all intersections that were discovered in the last loop.
    // only valid if any of the haptic shapes we have is dynamic.
    std::vector< PlaneConstraint > all_constraints;

    H3DUtil::MutexLock proxy_info_lock;
  };
}

#endif
