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
/// \file H3DTypeOperators.h
/// \brief Contains operators between different types in the ArithmeticTypes 
/// namespace.
///
//
//////////////////////////////////////////////////////////////////////////////

#ifndef __H3DTYPEOPERATORS_H__
#define __H3DTYPEOPERATORS_H__

#include "Vec2f.h"
#include "Vec2d.h"
#include "Vec3f.h"
#include "Vec3d.h"
#include "Vec4f.h"
#include "Vec4d.h"
#include "Rotation.h"
#include "Matrix3f.h"
#include "Matrix3d.h"
#include "Matrix4f.h"
#include "Matrix4d.h"
#include "Quaternion.h"

namespace H3D {
  ////////////////////////////////////////////////////////////////////

  /// DO NOT USE THIS NAMESPACE DIRECTLY. This namespace contains types
  /// that we want to perform arithmetic operations on. It contains
  /// template operators for most operations. The only operators a 
  /// type in this namespace has to define is the ==, +, * and / operator. 
  /// The rest will be defined through the template operators using the
  /// above mentioned operators.
  /// 
  namespace ArithmeticTypes {
				
    /// Multiplication between Matrix3f and Vec3f.
    /// \ingroup Matrix3fOperators
    /// \ingroup Vec3fOperators
    inline Vec3f operator*( const Matrix3f &m, const Vec3f &v ) {
      return Vec3f( m[0][0]*v.x + m[0][1]*v.y + m[0][2]*v.z,
		    m[1][0]*v.x + m[1][1]*v.y + m[1][2]*v.z,
		    m[2][0]*v.x + m[2][1]*v.y + m[2][2]*v.z );
    }


    /// Multiplication between Matrix3f and Vec3d.
    /// \ingroup Vec3dOperators
    /// \ingroup Matrix3fOperators
    inline Vec3d operator*( const Matrix3f &m, const Vec3d &v ) {
      return Vec3d( m[0][0]*v.x + m[0][1]*v.y + m[0][2]*v.z,
		    m[1][0]*v.x + m[1][1]*v.y + m[1][2]*v.z,
		    m[2][0]*v.x + m[2][1]*v.y + m[2][2]*v.z );
    }

    /// Multiplication between Matrix3d and Vec3d.
    /// \ingroup Vec3dOperators
    /// \ingroup Matrix3dOperators
    inline Vec3d operator*( const Matrix3d &m, const Vec3d &v ) {
      return Vec3d( m[0][0]*v.x + m[0][1]*v.y + m[0][2]*v.z,
                    m[1][0]*v.x + m[1][1]*v.y + m[1][2]*v.z,
                    m[2][0]*v.x + m[2][1]*v.y + m[2][2]*v.z );
    }


    /// Multiplication between Rotation and Vec3f is the Vec3f
    /// rotated by the Rotation.
    /// \ingroup RotationOperators
    /// \ingroup Vec3fOperators
    inline Vec3f operator*( const Rotation &r, const Vec3f &v ) {
      return (Matrix3f)r * v;
    }


    /// Multiplication between Rotation and Vec3d.
    /// \ingroup RotationOperators
    /// \ingroup Vec3dOperators
    inline Vec3d operator*( const Rotation &r, const Vec3d &v ) {
      return (Matrix3f)r * v;
    }

    /// Multiplication between Rotation and Vec3f is the Vec3f
    /// rotated by the Rotation.
    /// \ingroup QuaternionOperators
    /// \ingroup Vec3fOperators
    inline Vec3f operator*( const Quaternion &r, const Vec3f &v ) {
      return (Matrix3f)r * v;
    }

    /// Multiplication between Matrix3f and Vec3d.
    /// \ingroup QuaternionOperators
    /// \ingroup Vec3dOperators
    inline Vec3d operator*( const Quaternion &r, const Vec3d &v ) {
      return (Matrix3f)r * v;
    }
		
    /// Multiplication between Matrix4f and Vec4f.
    /// \ingroup Matrix4fOperators
    /// \ingroup Vec4fOperators
    inline Vec4f operator*( const Matrix4f &m, const Vec4f &v ) {
      return Vec4f( 
		   m[0][0]*v.x + m[0][1]*v.y + m[0][2]*v.z + m[0][3]*v.w,
		   m[1][0]*v.x + m[1][1]*v.y + m[1][2]*v.z + m[1][3]*v.w,
		   m[2][0]*v.x + m[2][1]*v.y + m[2][2]*v.z + m[2][3]*v.w,
		   m[3][0]*v.x + m[3][1]*v.y + m[3][2]*v.z + m[3][3]*v.w );
    }
				
    /// Multiplication between Matrix4f and Vec4d.
    /// \ingroup Matrix4fOperators
    /// \ingroup Vec4dOperators
    inline Vec4d operator*( const Matrix4f &m, const Vec4d &v ) {
      return Vec4d( 
		   m[0][0]*v.x + m[0][1]*v.y + m[0][2]*v.z + m[0][3]*v.w,
		   m[1][0]*v.x + m[1][1]*v.y + m[1][2]*v.z + m[1][3]*v.w,
		   m[2][0]*v.x + m[2][1]*v.y + m[2][2]*v.z + m[2][3]*v.w,
		   m[3][0]*v.x + m[3][1]*v.y + m[3][2]*v.z + m[3][3]*v.w );
    }

    /// Multiplication between Matrix4d and Vec4d.
    /// \ingroup Matrix4dOperators
    /// \ingroup Vec4dOperators
    inline Vec4d operator*( const Matrix4d &m, const Vec4d &v ) {
      return Vec4d( 
		   m[0][0]*v.x + m[0][1]*v.y + m[0][2]*v.z + m[0][3]*v.w,
		   m[1][0]*v.x + m[1][1]*v.y + m[1][2]*v.z + m[1][3]*v.w,
		   m[2][0]*v.x + m[2][1]*v.y + m[2][2]*v.z + m[2][3]*v.w,
		   m[3][0]*v.x + m[3][1]*v.y + m[3][2]*v.z + m[3][3]*v.w );
    }

    /// Multiplication between Matrix4f and Vec3f. It is assumed that the
    /// fourth element is 1. 
    /// \ingroup Matrix4fOperators
    /// \ingroup Vec3fOperators
    inline Vec3f operator*( const Matrix4f &m, const Vec3f &v ) {
      return Vec3f( m[0][0]*v.x + m[0][1]*v.y + m[0][2]*v.z + m[0][3],
                    m[1][0]*v.x + m[1][1]*v.y + m[1][2]*v.z + m[1][3],
                    m[2][0]*v.x + m[2][1]*v.y + m[2][2]*v.z + m[2][3] ) *
        ( 1 / ( m[3][0]*v.x + m[3][1]*v.y + m[3][2]*v.z + m[3][3] ) );
    }

    /// Multiplication between Matrix4f and Vec3d. It is assumed that the
    /// fourth element is 1. 
    /// \ingroup Matrix4fOperators
    /// \ingroup Vec3dOperators
    inline Vec3d operator*( const Matrix4f &m, const Vec3d &v ) {
      return Vec3d( m[0][0]*v.x + m[0][1]*v.y + m[0][2]*v.z + m[0][3],
                    m[1][0]*v.x + m[1][1]*v.y + m[1][2]*v.z + m[1][3],
                    m[2][0]*v.x + m[2][1]*v.y + m[2][2]*v.z + m[2][3] ) *
        ( 1 / ( m[3][0]*v.x + m[3][1]*v.y + m[3][2]*v.z + m[3][3] ) );
    }

    /// Multiplication between Matrix4d and Vec3d. It is assumed that the
    /// fourth element is 1. 
    /// \ingroup Matrix4dOperators
    /// \ingroup Vec3dOperators
    inline Vec3d operator*( const Matrix4d &m, const Vec3d &v ) {
      return Vec3d( m[0][0]*v.x + m[0][1]*v.y + m[0][2]*v.z + m[0][3],
                    m[1][0]*v.x + m[1][1]*v.y + m[1][2]*v.z + m[1][3],
                    m[2][0]*v.x + m[2][1]*v.y + m[2][2]*v.z + m[2][3] ) *
        ( 1 / ( m[3][0]*v.x + m[3][1]*v.y + m[3][2]*v.z + m[3][3] ) );
    }

  }
}


#endif
