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
/// \file FrictionSurface.h
/// \brief Header file for FrictionSurface
///
//
//////////////////////////////////////////////////////////////////////////////
#ifndef __FRICTIONSURFACE_H__
#define __FRICTIONSURFACE_H__

#include <HAPI/HAPIFrictionSurface.h>

namespace HAPI {
  /// \ingroup Surfaces
  /// \class FrictionSurface
  /// \brief A basic surface with friction capabilities.
  ///
  /// By default it behaves as if there is no friction at all on the surface.
  /// The force calculated always points from the probe towards the
  /// proxy. The force along the surface multiplied by the dynamic friction
  /// parameter must exceed the normal force of the surface multiplied by
  /// the static friction parameter in order for movement along the surface to
  /// start.
  class HAPI_API FrictionSurface : public HAPIFrictionSurface {
  public:

    /// Constructor
    FrictionSurface( HAPIFloat _stiffness = 0.35,
                     HAPIFloat _damping = 0,
                     HAPIFloat _static_friction = 0,
                     HAPIFloat _dynamic_friction = 0,
                     bool _use_relative_values = true,
                     bool _use_ref_count_lock = true );

    /// Moves the proxy along the surface in the direction of the
    /// probe. The friction parameters limit the movement and can
    /// sometimes stop the proxy from being moved at all.
    /// \param contact_info Contains information about the contact. The
    /// function must set the proxy movement by calling setLocalProxyMovement
    /// function.
    virtual void getProxyMovement( ContactInfo &contact_info );

    /// Calculate a force from the probe towards the proxy.
    virtual void getForces( ContactInfo &contact_info );
  };
}

#endif
