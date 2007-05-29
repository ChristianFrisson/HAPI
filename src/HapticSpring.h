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
/// \file HapticSpring.h
/// \brief Header file for HapticSpring.
///
//
//////////////////////////////////////////////////////////////////////////////
#ifndef __HAPTICSPRING_H__
#define __HAPTICSPRING_H__

#include <HAPIForceEffect.h> 
#include <HAPIHapticsDevice.h>

namespace HAPI {

  /// Generates a spring force, 
  /// i.e. force = ( position - device_position ) * spring_constant.
  /// 
  class HAPI_API HapticSpring: public HAPIForceEffect {
  public:
    /// Constructor
    HapticSpring ( const Matrix4 & _transform,
                   bool _interpolate ):
      HAPIForceEffect( _transform, _interpolate ),
      position( Vec3( 0, 0, 0 ) ),
      spring_constant( 0 ) { }
    
    /// Constructor
    HapticSpring( const Matrix4 & _transform,
                  const Vec3 &_position,
                  HAPIFloat _spring_constant,
                  bool _interpolate );
    
    /// The force of the EffectOutput will be the force of the force field. 
    EffectOutput virtual calculateForces( HAPIHapticsDevice *hd,
                                          HAPITime dt ) {
      //lock.lock();
      Vec3 local_pos = transform.inverse() * hd->getPosition();
      Vec3 local_force = ( position - local_pos ) * spring_constant;
      force = local_force;
      //lock.unlock();
      return EffectOutput( transform.getRotationPart() * local_force );
    }

    // set position
    inline void setPosition( const Vec3 &_position ) { 
      //lock.lock();
      position = _position;
      //lock.unlock();
    }

    // set velocity
    inline void setVelocity( const Vec3 &_velocity ) {
      //lock.lock();
      velocity = _velocity;
      //lock.unlock();
    }

    // set velocity
    inline void setSpringConstant( const HAPIFloat &_sc ) { 
      //lock.lock();
      spring_constant = _sc;
      //lock.unlock();
    }
    
    // get and reset force
    inline Vec3 getLatestForce() {
      //lock.lock();
      Vec3 f(force);
      //lock.unlock();
      return f;
    }
    
  protected:
    Vec3 position;
    Vec3 velocity;
    Vec3 force;
    HAPIFloat spring_constant;
    //MutexLock lock;
  };
}

#endif
