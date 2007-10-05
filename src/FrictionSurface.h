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
/// \file FrictionSurface.h
/// \brief Header file for FrictionSurface
///
//
//////////////////////////////////////////////////////////////////////////////
#ifndef __FRICTIONSURFACE_H__
#define __FRICTIONSURFACE_H__

#include <HAPISurfaceObject.h>

namespace HAPI {
  /// FrictionSurface is basic surface with friction capabilities.
  /// By default it behaves as if there is no friction at all on the surface.
  /// The force calculated always points from the probe towards the
  /// proxy. The force along the surface multiplied by the dynamic friction
  /// parameter must exceed the normal force of the surface multiplied by
  /// the static friction parameter in order for movement along the surface to
  /// start.
  class HAPI_API FrictionSurface : public HAPISurfaceObject {
  public:

    /// Constructor
    FrictionSurface( HAPIFloat _stiffness = 0.35,
                    HAPIFloat _damping = 0,
                    HAPIFloat _static_friction = 0,
                    HAPIFloat _dynamic_friction = 0,
                    bool _use_ref_count_lock = true );

    /// ~Destructor
    virtual ~FrictionSurface();

    /// Moves the proxy along the surface in the direction of the
    /// probe. The friction parameters limit the movement and can
    /// sometimes stop the proxy from being moved at all.
    virtual void getProxyMovement( ContactInfo &contact_info );

    /// Calculate a force from the probe towards the proxy.
    virtual void getForces( ContactInfo &contact_info );

    /// The stiffness of the surface.
    HAPIFloat stiffness;

    /// The damping parameter of the surface.
    HAPIFloat damping;
    
    /// The static friction of the surface.
    HAPIFloat static_friction;
    
    /// The dynamic friction of the surface
    HAPIFloat dynamic_friction;
  protected:
    /// Used to determine if the proxy is in static contact with
    /// the surface. In that case there is no proxy movement.
    bool in_static_contact;
  };
}

#endif
