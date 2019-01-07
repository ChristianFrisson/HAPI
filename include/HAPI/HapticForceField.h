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
/// \file HapticForceField.h
/// \brief Header file for HapticForceField
///
//
//////////////////////////////////////////////////////////////////////////////
#ifndef __HAPTICFORCEFIELD_H__
#define __HAPTICFORCEFIELD_H__

#include <HAPI/HAPIForceEffect.h> 

namespace HAPI {
  /// \ingroup ForceEffects
  /// \class HapticForceField
  /// \brief A haptic force effect that generates a constant force.
  class HAPI_API HapticForceField: public HAPIForceEffect {
  public:

    /// Constructor.
    HapticForceField( const Vec3 &_force,
                      const Vec3 &torque = Vec3( 0, 0, 0 ),
                      bool _use_ref_count_lock = false );

    
    /// The force of the EffectOutput will be the force of the force field. 
    EffectOutput virtual calculateForces( const EffectInput &input ) {
      return EffectOutput( force, torque );
    }
    
  protected:
    Vec3 force, torque;
  };
}

#endif
