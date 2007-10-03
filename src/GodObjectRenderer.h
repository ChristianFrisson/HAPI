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
/// \file GodObjectRenderer.h
/// \brief Header file for GodObjectRenderer.
///
//
//////////////////////////////////////////////////////////////////////////////

#ifndef __GODOBJECTRENDERER_H__
#define __GODOBJECTRENDERER_H__

#include <HAPIProxyBasedRenderer.h>
#include <Threads.h>
#include <map>

namespace HAPI {

  /// Haptics renderer that uses a variant of the algorithm presented
  /// in the paper "A Constraint-based God-object Method For Haptic Display"
  /// by C. B. Zilles J. K. Salisbury.
  ///
  /// The god object is a point that stays on the geometry's surface when the
  /// probe penetrates the surface(which in our implementation is called 
  /// a proxy). This renderer only allows for point proxies.  
  class HAPI_API GodObjectRenderer: public HAPIProxyBasedRenderer {
  public:
    
    /// Destructor.
    virtual ~GodObjectRenderer() {}

    /// The main function in any haptics renderer. Given a haptics device and 
    /// a group of shapes, it generates the force and torque to send to the
    /// device.
    virtual HAPIForceEffect::EffectOutput 
    renderHapticsOneStep(  HAPIHapticsDevice *hd,
                          const HapticShapeVector &shapes );

    /// Get the current position of the proxy.
    inline virtual Vec3 getProxyPosition() {
      return proxy_position;
    }

    /// Register this renderer to the haptics renderer database.
    static HapticsRendererRegistration renderer_registration;

  protected:
    void onOnePlaneContact( const PlaneConstraint &c, 
                            HAPISurfaceObject::ContactInfo &contact,
                            const HapticShapeVector &shapes );

    void onTwoPlaneContact( const PlaneConstraint &p0,
                            const PlaneConstraint &p1,
                            HAPISurfaceObject::ContactInfo &contact,
                            const HapticShapeVector &shapes );

    void onThreeOrMorePlaneContact(  vector< PlaneConstraint > &constraints,
                                     HAPISurfaceObject::ContactInfo &contact,
                                     const HapticShapeVector &shapes );

    bool tryProxyMovement( Vec3 from, Vec3 to, 
                           int nr_constraints,
                           const PlaneConstraint &pc,
                           const HapticShapeVector &shapes,
                           Vec3 &point );
    
    Vec3 proxy_position;
    Contacts tmp_contacts;

  };
}

#endif
