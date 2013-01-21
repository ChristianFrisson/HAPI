//////////////////////////////////////////////////////////////////////////////
//    Copyright 2004-2013, SenseGraphics AB
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
/// \file HapticSpring.h
/// \brief Header file for HapticSpring.
///
//
//////////////////////////////////////////////////////////////////////////////
#ifndef __HAPTICSPRING_H__
#define __HAPTICSPRING_H__

#include <HAPI/HAPIForceEffect.h> 
#include <HAPI/HAPIHapticsDevice.h>

namespace HAPI {

  /// \ingroup ForceEffects
  /// \class HapticSpring
  /// \brief Generates a spring force, 
  /// i.e. force = ( position - device_position ) * spring_constant - damping * device_velocity.
  class HAPI_API HapticSpring: public HAPIForceEffect {
  public:
    /// Constructor
    HapticSpring():
      position( Vec3( 0, 0, 0 ) ),
      spring_constant( 0 ),
      damping( 0 ) { }
    
    /// Constructor
    HapticSpring( const Vec3 &_position,
                  HAPIFloat _spring_constant );

    HapticSpring( const Vec3 &_position,
                  HAPIFloat _spring_constant,
                  HAPIFloat _damping );
    
    /// The force of the EffectOutput will be a force from the position of
    /// the haptics device to the position of the HapticSpring. 
    EffectOutput virtual calculateForces( const EffectInput &input ) {
      force = 
        ( position - input.hd->getPosition() ) * spring_constant - 
        damping * input.hd->getVelocity();
      return EffectOutput( force );
    }

    /// Set position
    inline void setPosition( const Vec3 &_position ) {
      position = _position;
    }

    /// Set damping
    inline void setDamping( HAPIFloat _damping ) {
      damping = _damping;
    }
    
    /// Set spring constant
    inline void setSpringConstant( const HAPIFloat &_sc ) {
      spring_constant = _sc;
    }

    /// Get position
    inline const Vec3 &getPosition() {
      return position;
    }

    /// Get damping
    inline HAPIFloat getDamping() {
      return damping;
    }

    /// Get spring constant
    inline HAPIFloat getSpringConstant() {
      return spring_constant;
    }

    /// Get last force
    inline const Vec3 &getLatestForce() {
      return force;
    }
    
  protected:
    Vec3 force;
    Vec3 position;
    HAPIFloat spring_constant;
    HAPIFloat damping;
  };
}

#endif
