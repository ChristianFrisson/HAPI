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

  /// Haptics renderer that uses a variant of the algorithm presented
  /// in the paper "The haptic display of complex graphical environments"
  /// by Diego C. Ruspini, Krasimir Kolarov and Oussama Khatib.
  ///
  /// This renderer allows for variable proxy radius, i.e. a sphere proxy.
  class HAPI_API RuspiniRenderer: public HAPIProxyBasedRenderer {
  public:
	  typedef Collision::CollisionObject::Constraints Constraints;

    /// Constructor.
    RuspiniRenderer( HAPIFloat _proxy_radius = 2.5 );

    /// Destructor.
    virtual ~RuspiniRenderer() {}

   /// The main function in any haptics renderer. Given a haptics device and 
    /// a group of shapes generate the force and torque to send to the device.
    virtual HAPIForceEffect::EffectOutput 
    renderHapticsOneStep(  HAPIHapticsDevice *hd,
                          const HapticShapeVector &shapes );

    /// Get the current position of the proxy.
    inline virtual Vec3 getProxyPosition() {
      return proxy_position;
    }

    /// Get the current radius of the proxy.
    inline HAPIFloat getProxyRadius() {
      return proxy_radius;
    }

    /// Set the radius of the proxy(in millimetres)
    inline void setProxyRadius( HAPIFloat r ) {
      proxy_radius = r;
    }
    
    /// Register this renderer to the haptics renderer database.
    static HapticsRendererRegistration renderer_registration;

  protected:
    void onOnePlaneContact( const PlaneConstraint &c, 
                            HAPISurfaceObject::ContactInfo &contact );

    void onTwoPlaneContact( const PlaneConstraint &p0,
                            const PlaneConstraint &p1,
                            HAPISurfaceObject::ContactInfo &contact );

    void onThreeOrMorePlaneContact(  Constraints &constraints,
                                     HAPISurfaceObject::ContactInfo &contact );

    Vec3 tryProxyMovement( Vec3 from, Vec3 to, Vec3 normal );
    
    HAPIFloat proxy_radius;
    Vec3 proxy_position;
    Contacts tmp_contacts;

    Constraints constraints;
    Constraints closest_constraints;
    Constraints other_constraints;

    
  };
}

#endif
