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
    HapticSpring( bool _use_ref_count_lock = false ):
      HAPIForceEffect( _use_ref_count_lock ),
      position( Vec3( 0, 0, 0 ) ),
      spring_constant( 0 ),
      damping( 0 ),
      first_time( true ),
      step_length( 0 ),
      position_interpolation( 1 ) { }
    
    /// Constructor
    HapticSpring( const Vec3 &_position,
                  HAPIFloat _spring_constant,
                  bool _use_ref_count_lock = false );

    HapticSpring( const Vec3 &_position,
                  HAPIFloat _spring_constant,
                  HAPIFloat _damping,
                  HAPIFloat _position_interpolation = 1,
                  bool _use_ref_count_lock = false );
    
    /// The force of the EffectOutput will be a force from the position of
    /// the haptics device to the position of the HapticSpring. 
    EffectOutput virtual calculateForces( const EffectInput &input );

    /// Set position
    inline void setPosition( const Vec3 &_position ) {
      Vec3 difference= first_time ? (_position - position) : (_position - interpolated_position);
      step_length = difference.length() * position_interpolation;
      position = _position;
    }

    /// Set damping
    inline void setDamping( HAPIFloat _damping ) {
      damping = _damping;
    }
    
    /// Set spring constant
    inline void setSpringConstant( HAPIFloat _sc ) {
      spring_constant = _sc;
    }

    /// Set position_interpolation constant
    inline void setPositionInterpolation( HAPIFloat _pi ) {
      position_interpolation = _pi;
      if( position_interpolation < 0 )
        position_interpolation = 0;
      else if( position_interpolation > 1 )
        position_interpolation = 1;
    }

    /// Get position
    inline const Vec3 &getPosition() {
      return position;
    }

    /// Get the interpolated spring position
    inline const Vec3 &getInterpolatedPosition() {
      return interpolated_position;
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

    /// Get position_interpolation.
    inline HAPIFloat getPositionInterpolation() {
      return position_interpolation;
    }
    
  protected:
    Vec3 force, position, interpolated_position, last_set_position;
    HAPIFloat spring_constant, damping, step_length, position_interpolation;
    bool first_time;
  };
}

#endif
