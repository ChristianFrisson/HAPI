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
/// \file HapticShapeConstraint.h
/// \brief Header file for HapticShapeConstraint
///
//
//////////////////////////////////////////////////////////////////////////////
#ifndef __HAPTICSHAPECONSTRAINT_H__
#define __HAPTICSHAPECONSTRAINT_H__

#include <HAPI/HAPIForceEffect.h> 
#include <HAPI/HAPIHapticsDevice.h>
#include <HAPI/HAPIHapticShape.h>

namespace HAPI {
  /// \ingroup ForceEffects
  /// \class HapticShapeConstraint
  /// \brief A haptic force effect which constrain the haptic device to a
  /// shape.
  ///
  /// The force calculated is a spring force from the haptic
  /// device position to the closest point of the given CollisionObject. The
  /// CollisionObject could for example be a HAPIHapticShape.
  class HAPI_API HapticShapeConstraint: public HAPIForceEffect {
  public:
    /// Constructor
    HapticShapeConstraint( Collision::CollisionObject *_col_obj,
                           const HAPIFloat &_spring_constant,
                           bool _use_ref_count_lock = false );
    
    /// The force of the EffectOutput is a spring force from the current
    /// position of the haptics device towards the closest point on the
    /// CollisionObject.
    EffectOutput virtual calculateForces( const EffectInput &input );

  protected:
    H3DUtil::AutoRef< Collision::CollisionObject > col_obj;
    HAPIFloat spring_constant;
  };
}

#endif
