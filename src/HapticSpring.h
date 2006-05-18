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
/// \file HapticSpring.h
/// \brief Header file for HapticSpring.
///
//
//////////////////////////////////////////////////////////////////////////////
#ifndef __HAPTICSPRING_H__
#define __HAPTICSPRING_H__

#include <HapticForceEffect.h> 

namespace H3D {

  /// Generates a spring force, 
  /// i.e. force = ( position - device_position ) * spring_constant.
  /// 
  class HAPI_API HapticSpring: public HapticForceEffect {
  public:
    /// Constructor
    HapticSpring ( const H3D::ArithmeticTypes::Matrix4f & _transform,
                   bool _interpolate ):
      HapticForceEffect( _transform, _interpolate ),
      position( Vec3f( 0, 0, 0 ) ),
      spring_constant( 0 ) { }
    
    /// Constructor
    HapticSpring( const H3D::ArithmeticTypes::Matrix4f & _transform,
                  const Vec3f &_position,
                  H3DFloat _spring_constant,
                  bool _interpolate );
    
    /// The force of the EffectOutput will be the force of the force field. 
    EffectOutput virtual calculateForces( const EffectInput &input ) {
      //lock.lock();
      Vec3d local_pos = transform.inverse() * input.position;
      Vec3d local_force = ( position - local_pos ) * spring_constant;
      force = local_force;
      //lock.unlock();
      return EffectOutput( transform.getRotationPart() * local_force );
    }

    // set position
    inline void setPosition( const Vec3f &_position ) { 
      //lock.lock();
      position = _position;
      //lock.unlock();
    }

    // set velocity
    inline void setVelocity( const Vec3f &_velocity ) {
      //lock.lock();
      velocity = _velocity;
      //lock.unlock();
    }

    // set velocity
    inline void setSpringConstant( const H3DFloat &_sc ) { 
      //lock.lock();
      spring_constant = _sc;
      //lock.unlock();
    }
    
    // get and reset force
    inline Vec3f getLatestForce() {
      //lock.lock();
      Vec3f f(force);
      //lock.unlock();
      return f;
    }
    
  protected:
    Vec3f position;
    Vec3f velocity;
    Vec3d force;
    H3DFloat spring_constant;
    //MutexLock lock;
  };
}

#endif
