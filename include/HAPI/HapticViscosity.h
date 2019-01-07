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
  /// \ingroup ForceEffects
  /// \class HapticViscosity
  /// \brief A haptic force effect which tries to simulate viscosity.
  ///
  /// HapticViscosity uses stokes law to calculate the force
  /// exerted on a sphere of radius r in a fluid with a certain viscosity.
  /// The flow is assumed to be laminar. It will only be valid for fairly small
  /// values of viscosity and radius. Also the speed of the haptics device can
  /// not be to high. The algorithm is very simple all it does is calculating
  /// a force in the opposite direction of the velocity of the haptics device.
  class HAPI_API HapticViscosity: public HAPIForceEffect {
  public:
    /// Constructor
    /// \param _viscosity is the viscosity in Pas
    /// \param _radius is the radius of the sphere in meters.
    /// \param _damping_factor is best left as its initial value unless
    /// there is a need to stop vibrations that comes from a to large
    /// constant e.g. when the force calculated exceeds the force inputed by
    /// the user when moving the proxy.
    /// \param _use_ref_count_lock If true, the force effect will have lock 
    ///                            protected ref counting to guarantee its thread safety
    HapticViscosity( const HAPIFloat &_viscosity,
                     const HAPIFloat &_radius,
                     const HAPIFloat &_damping_factor = 1,
                     bool _use_ref_count_lock = false );
    
    /// The force of the EffectOutput will be calculated using stokes law.
    EffectOutput virtual calculateForces( const EffectInput &input );
    
  protected:
    // viscosity is the viscosity in Pas
    HAPIFloat viscosity;
    // radius is the radius of the sphere in meters.
    HAPIFloat radius;
    // _damping_factor is best left as its initial value unless
    HAPIFloat the_constant;
  };
}

#endif
