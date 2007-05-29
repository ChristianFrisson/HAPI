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
/// \file HapticShapeConstraint.h
/// \brief Header file for HapticShapeConstraint
///
//
//////////////////////////////////////////////////////////////////////////////
#ifndef __HAPTICSHAPECONSTRAINT_H__
#define __HAPTICSHAPECONSTRAINT_H__

#include <HAPIForceEffect.h> 
#include <HAPIHapticsDevice.h>
#include <HAPIHapticShape.h>

namespace HAPI {
  /// Adds a force to constrain the haptic device position to the surface.
  class HAPI_API HapticShapeConstraint: public HAPIForceEffect {
  public:
    /// Constructor
    HapticShapeConstraint( const Matrix4 & _transform,
                          bool _interpolate,
                          HAPIHapticShape *_shape,
                          const HAPIFloat &_spring_constant);
    
    /// The force of the EffectOutput is a spring force from the current
    /// position of the haptics device towards the closest point on the
    /// haptic shape.
    EffectOutput virtual calculateForces( HAPIHapticsDevice *hd,
                                          HAPITime dt );

  protected:
    H3DUtil::AutoRef< HAPIHapticShape > shape;
    HAPIFloat spring_constant;
  };
}

#endif
