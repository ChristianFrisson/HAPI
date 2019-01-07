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
/// \file HapticPositionFunctionEffect.h
/// \brief Header file for HapticPositionFunctionEffect
///
//
//////////////////////////////////////////////////////////////////////////////
#ifndef __HAPTICPOSITIONFUNCTIONEFFECT_H__
#define __HAPTICPOSITIONFUNCTIONEFFECT_H__

#include <HAPI/HAPIForceEffect.h> 
#include <HAPI/HAPIHapticsDevice.h>
#include <HAPI/HAPIFunctionObject.h>

namespace HAPI {
  /// \ingroup ForceEffects
  /// \class HapticPositionFunctionEffect
  /// \brief A haptic force effect which creates a force that depends on the
  /// position of the device.
  ///
  /// The force is calculated by evaluating three
  /// functions ( one for each dimension ) with the device position as input
  /// ( parameters x, y, z ).
  /// It is up to the user to make sure that the functions provided does not
  /// create a force that the attached device can not handle.
  class HAPI_API HapticPositionFunctionEffect: public HAPIForceEffect {
  public:
    /// Constructor
    /// ownership of the pointer in the parameters _x_function, _y_function and
    /// _z_function are transferred to the HapticPositionFunctionEffect.
    HapticPositionFunctionEffect( 
                          HAPIFunctionObject *_x_function,
                          HAPIFunctionObject *_y_function,
                          HAPIFunctionObject *_z_function,
                          bool _use_ref_count_lock = false );
    
    /// The force of the EffectOutput is calculated from the provided functions
    EffectOutput virtual calculateForces( const EffectInput &input );

  protected:
    std::auto_ptr< HAPIFunctionObject > x_function;
    std::auto_ptr< HAPIFunctionObject > y_function;
    std::auto_ptr< HAPIFunctionObject > z_function;
  };
}

#endif
