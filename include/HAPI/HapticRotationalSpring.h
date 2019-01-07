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
/// \file HapticRotationalSpring.h
/// \brief Header file for HapticRotationalSpring.
///
//
//////////////////////////////////////////////////////////////////////////////
#ifndef __HAPTICROTATIONALSPRING_H__
#define __HAPTICROTATIONALSPRING_H__

#include <HAPI/HAPIForceEffect.h> 
#include <HAPI/HAPIHapticsDevice.h>

namespace HAPI {

  /// \ingroup ForceEffects
  /// \class HapticRotationalSpring
  /// \brief Generates a spring torque.
  ///
  /// The torque is generated using the following formula.
  /// i.e. torque = rotation_diff_euler_angles * spring_constant -
  /// damping * device_angular_velocity.
  /// The rotation_diff_angle is the euler angle for the rotation from the
  /// current axis to the desired axis
  /// and the default axis (0, 0, 1) rotated by the device rotation. The
  /// spring_constant is a constant given in
  /// Nm/radians. The damping is a constant given in Nms/radians and the
  /// device_angular_velocity is the angular velocity of the device.
  /// \note AT THE MOMENT ANGULAR VELOCITY AND DAMPING ARE IGNORED.
  class HAPI_API HapticRotationalSpring: public HAPIForceEffect {
  public:
    /// Constructor
    HapticRotationalSpring( bool _use_ref_count_lock = false ):
      HAPIForceEffect( _use_ref_count_lock ),
      desired_axis( 0, 0, 1 ),
      spring_constant( 0 ),
      damping( 0 ) { }

    /// Constructor
    HapticRotationalSpring( const Vec3 &_desired_axis,
                            HAPIFloat _spring_constant,
                            bool _use_ref_count_lock = false );

    /// Constructor
    HapticRotationalSpring( const Vec3 &_desired_axis,
                            HAPIFloat _spring_constant,
                            HAPIFloat _damping,
                            bool _use_ref_count_lock = false );

    /// The force of the EffectOutput will be a torque from the current device
    /// rotation to the desired rotation.
    EffectOutput virtual calculateForces( const EffectInput &input ) {
      Quaternion result( Rotation( input.hd->getOrientation() * Vec3( 0, 0, 1 ), desired_axis ) );
      torque = spring_constant * result.v;
      return EffectOutput( Vec3(), torque );
    }

    /// Set rotation
    inline void setDesiredAxis( const Vec3 &_desired_axis ) { 
      desired_axis = _desired_axis;
    }

    /// Set damping
    inline void setDamping( HAPIFloat _damping ) {
      damping = _damping;
    }
    
    /// Set spring constant
    inline void setSpringConstant( const HAPIFloat &_sc ) {
      spring_constant = _sc;
    }

    /// Get rotation
    inline const Vec3 &getDesiredAxis() {
      return desired_axis;
    }

    /// Get damping
    inline HAPIFloat getDamping() {
      return damping;
    }

    /// Get spring constant
    inline HAPIFloat getSpringConstant() {
      return spring_constant;
    }

    // Get last torque
    inline const Vec3 &getLatestTorque() {
      return torque;
    }

  protected:
    Vec3 torque, desired_axis;
    HAPIFloat spring_constant;
    HAPIFloat damping;
  };
}

#endif
