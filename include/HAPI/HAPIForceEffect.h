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

  /// \ingroup AbstractClasses
  /// \class HAPIForceEffect
  /// \brief The base class for force effects.
  ///
  /// A HAPIForceEffect is a class that generates force and torque based
  /// values of the haptics device such as position, orientation and velocity.
  /// It is a good choice to use to create global effects such as force
  /// fields, springs, recoil effects and wind.
  class HAPI_API HAPIForceEffect: public HAPIHapticObject, 
                                  public H3DUtil::RefCountedClass {
  public:

    /// The input to the calculateForces function of HAPIForceEffect. 
    struct HAPI_API EffectInput {
      /// Constructor.
      EffectInput( HAPIHapticsDevice *_hd,
                   const TimeStamp &dt = 0 );
      
      /// The position of the finger. THIS VARIABLE IS
      /// DEPRECATED, use hd->getPosition instead.
      Vec3 position;
      /// The velocity of the finger. THIS VARIABLE IS
      /// DEPRECATED, use hd->getVelocity instead.
      Vec3 velocity;
      /// The orientation of the stylus. THIS VARIABLE IS
      /// DEPRECATED, use hd->getOrientation instead.
      Rotation orientation;
      /// The change in time since the last call
      TimeStamp deltaT;
      /// The haptics device for the device that created the EffectInput
      HAPIHapticsDevice *hd;
    };

    /// The output from the calculateForces function of HAPIForceEffect.
    struct HAPI_API EffectOutput {
      /// Constructor.
      EffectOutput( const Vec3 _force = Vec3( 0,0,0 ),
                    const Vec3 _torque = Vec3( 0,0,0 ),
                    const HAPIFloat _dof7_force = 0 ) :
        force( _force ),
        torque( _torque ),
        dof7_force( _dof7_force ) {}
      
      /// The force to render.
      Vec3 force;
      /// The torque to render.
      Vec3 torque;

      /// The torque to render.
      HAPIFloat dof7_force;
      
      /// Addition operator. Adds the forces and torques of the
      /// two EffectOutput instances.
      EffectOutput operator +( const EffectOutput &o ) {
        return EffectOutput( force + o.force,
                             torque + o.torque,
                             dof7_force + o.dof7_force );
      }

      /// Multiplication by float operator. Scales the forces and
      /// torque.
      EffectOutput operator *( float f ) {
        return EffectOutput( force * f,
                             torque *f,
                             dof7_force * f );
      }

      /// Multiplication by double operator. Scales the forces and
      /// torque.
      EffectOutput operator *( double f ) {
        return EffectOutput( force * f,
                             torque *f,
                             dof7_force * f );
      }
    };
    
    /// Constructor.
    HAPIForceEffect( bool _use_ref_count_lock = true ) : RefCountedClass( _use_ref_count_lock ){}
    
    /// The function that calculates the forces given by this 
    /// HAPIForceEffect.
    /// \param input Contains useful information, see EffectInput struct.
    /// \returns An EffectOutput struct that contains force and torque.
    EffectOutput virtual calculateForces( const EffectInput &input ) = 0;
    
    /// Destructor. Virtual to make HAPIForceEffect a polymorphic type.
    virtual ~HAPIForceEffect() {}
  };
}

#endif
