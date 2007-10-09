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

#include <HAPI/HAPIForceEffect.h> 
#include <HAPI/HAPIHapticsDevice.h>

namespace HAPI {

  /// \brief Generates a spring force, 
  /// i.e. force = ( position - device_position ) * spring_constant.
  /// 
  class HAPI_API HapticSpring: public HAPIForceEffect {
  public:
    /// Constructor
    HapticSpring ( bool _interpolate ):
      HAPIForceEffect( _interpolate ),
      position( Vec3( 0, 0, 0 ) ),
      spring_constant( 0 ) { }
    
    /// Constructor
    HapticSpring( const Vec3 &_position,
                  HAPIFloat _spring_constant,
                  bool _interpolate = false );
    
    /// The force of the EffectOutput will be the force of the force field. 
    EffectOutput virtual calculateForces( HAPIHapticsDevice *hd,
                                          HAPITime dt ) {
      force = ( position - hd->getPosition() ) * spring_constant;
      return EffectOutput( force );
    }

    // set position
    inline void setPosition( const Vec3 &_position ) { 
      position = _position;
    }
    
    // set velocity
    inline void setSpringConstant( const HAPIFloat &_sc ) { 
      spring_constant = _sc;
    }

    // get position
    inline const Vec3 &getPosition() { 
      return position;
    }

    // get spring constant
    inline HAPIFloat setSpringConstant() { 
      return spring_constant;
    }

    // get last force
    inline const Vec3 &getLatestForce() { 
      return force;
    }
    
  protected:
    Vec3 force;
    Vec3 position;
    HAPIFloat spring_constant;
  };
}

#endif
