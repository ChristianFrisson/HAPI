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
/// \file HapticViscosity.h
/// \brief Header file for HapticViscosity
///
//
//////////////////////////////////////////////////////////////////////////////
#ifndef __HAPTICVISCOSITY_H__
#define __HAPTICVISCOSITY_H__

#include <HAPI/HAPIForceEffect.h> 
#include <HAPI/HAPIHapticsDevice.h>

namespace HAPI {
  /// Uses stokes law to generate the force exerted on a sphere of radius r
  /// in a fluid with a certain viscosity. The flow is assumed to be laminar.
  class HAPI_API HapticViscosity: public HAPIForceEffect {
  public:
    /// Constructor
    HapticViscosity( const HAPIFloat &_viscosity,
                     const HAPIFloat &_radius,
                     const HAPIFloat &_damping_factor );
    
    /// The force of the EffectOutput will be calculated using stokes law
    EffectOutput virtual calculateForces( HAPIHapticsDevice *hd,
                                          HAPITime dt );
    
  protected:
    HAPIFloat viscosity;
    HAPIFloat radius;
    HAPIFloat the_constant;
  };
}

#endif
