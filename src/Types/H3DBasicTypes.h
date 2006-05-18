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
/// \file H3DBasicTypes.h
/// \brief Contains type definitions for the simple types in H3DAPI.
///
//
//////////////////////////////////////////////////////////////////////////////

#ifndef __H3DBASICTYPES_H__
#define __H3DBASICTYPES_H__
#include <limits>

namespace H3D {
  /// The floating point type.
  typedef float H3DFloat;
  /// Used for floating point values when specified as high-precision
  /// in the X3D specification.
  typedef double H3DDouble;
  /// The 32 bit integer type.
  typedef int H3DInt32;
  /// The type for time values.
  typedef double H3DTime;

  namespace Constants {
    /// The epsilon value to use for H3DFloat values.
    static const H3DFloat f_epsilon =  
    std::numeric_limits< H3DFloat >::epsilon();
    /// The epsilon value to use for H3DDouble values.
    static const H3DDouble d_epsilon = 
    std::numeric_limits< H3DDouble >::epsilon();
  }
}
#endif
