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
/// \file HAPIForceEffect.h
/// \brief Header file for HAPIForceEffect
///
//
//////////////////////////////////////////////////////////////////////////////
#ifndef __HAPIFORCEEFFECT_H__
#define __HAPIFORCEEFFECT_H__

#include <HAPI/HAPIHapticObject.h> 
#include <H3DUtil/RefCountedClass.h>
#include <H3DUtil/TimeStamp.h>

namespace HAPI {
  // forward declaration
  class HAPIHapticsDevice;

  /// The base class for force effects. A HAPIForceEffect is a class that 
  /// generates force and torque based values of the haptics device such
  /// as position, orientation and velocity. It is a good choice to use
  /// to create global effects such as force fields, springs, recoil
  /// effects and wind. By setting the HAPIForceEffect to be interpolated
  /// the HAPIForceEffect will be interpolated with the HAPIForceEffect
  /// added in the last scene-graph loop in order to avoid haptic rendering
  /// artifacts.
  class HAPI_API HAPIForceEffect: public HAPIHapticObject, 
                                  public H3DUtil::RefCountedClass {
  public:
    /// The output from a HAPIForceEffect.
    struct HAPI_API EffectOutput {
      /// Constructor.
      EffectOutput( const Vec3 _force = Vec3( 0,0,0 ),
                    const Vec3 _torque = Vec3( 0,0,0 ) ) :
        force( _force ),
        torque( _torque ) {}
      
      /// The force to render.
      Vec3 force;
      /// The torque to render.
      Vec3 torque;
      
      /// Addition operator. Adds the forces and torques of the
      /// two EffectOutput instances.
      EffectOutput operator +( const EffectOutput &o ) {
        return EffectOutput( force + o.force,
                             torque + o.torque );
      }

      /// Multiplication by float operator. Scales the forces and
      /// torque.
      EffectOutput operator *( float f ) {
        return EffectOutput( force * f,
                             torque *f );
      }

      /// Multiplication by double operator. Scales the forces and
      /// torque.
      EffectOutput operator *( double f ) {
        return EffectOutput( force * f,
                             torque *f );
      }
    };
    
    /// Constructor.
    HAPIForceEffect(){}
    
    /// The function that calculates the forces given by this 
    /// HAPIForceEffect.
    EffectOutput virtual calculateForces( HAPIHapticsDevice *hd,
                                          HAPITime dt ) = 0;
    
    /// Destructor. Virtual to make HAPIForceEffect a polymorphic type.
    virtual ~HAPIForceEffect() {}
  };
}

#endif
