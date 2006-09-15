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
/// \file HAPIHapticObject.h
/// \brief Header file for HapticObject
///
//
//////////////////////////////////////////////////////////////////////////////
#ifndef __HAPIHAPTICOBJECT_H__
#define __HAPIHAPTICOBJECT_H__

#include <HAPI.h>
#include <H3DTypes.h>

namespace H3D {

  /// The base class for all haptic objects. A haptic object performs
  /// any kind of haptic rendering e.g. rendering of shapes or
  /// general force effects.
  
  class HAPI_API HAPIHapticObject {
  public:
    /// Constructor
    HAPIHapticObject( const H3D::ArithmeticTypes::Matrix4d & _transform ):
      transform( _transform ) {}

    /// Destructor. Virtual to make HapticObject a polymorphic type.
    virtual ~HAPIHapticObject() {}

    /// Matrix transforming from local space to global space.
    Matrix4d transform;
  };
}

#endif
