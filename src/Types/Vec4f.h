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
/// \file Vec4f.h
/// \brief Header file for Vec4f.
///
//
//////////////////////////////////////////////////////////////////////////////

#ifndef __VEC4F_H__
#define __VEC4F_H__

#include "HAPI.h"
#include "H3DBasicTypes.h"
#include "H3DTemplateOperators.h"
#include "Exception.h"
#include <ostream>
using namespace std;

namespace H3D {
  namespace ArithmeticTypes {
    struct Vec4d;

    /// A Vec4f specifies a 4d vector. The values of the 
    /// Vec4f are X3Float.
    /// \ingroup BasicTypes.
    struct HAPI_API Vec4f {
      /// Default constructor.
      Vec4f(): x(0), y(0), z(0), w(1){}

      /// Constructor.
      Vec4f( H3DFloat _x,
             H3DFloat _y,
             H3DFloat _z,
             H3DFloat _w = 1 ) : x(_x), y(_y), z(_z), w(_w) {}

      /// Conversion from Vec4d.
      explicit Vec4f( const Vec4d &v );

       /// Return the components by their index, 
      /// x has index 0
      /// y has index 1
      /// z has index 2
      /// w has index 3
      inline H3DFloat &operator[]( int i ) { 
        if( i == 0 ) return x;
        if( i == 1 ) return y;
        if( i == 2 ) return z;
        if( i == 3 ) return w;
        
        throw Exception::H3DAPIException( "Invalid index", 
                                          H3D_FULL_LOCATION );
      }

      /// Return the components by their index, 
      /// x has index 0
      /// y has index 1
      /// z has index 2
      /// w has index 3
      inline const H3DFloat &operator[]( int i ) const { 
        if( i == 0 ) return x;
        if( i == 1 ) return y;
        if( i == 2 ) return z;
        if( i == 3 ) return w;
        
        throw Exception::H3DAPIException( "Invalid index", 
                                          H3D_FULL_LOCATION );
      }

      /// The public values of the vector.
      H3DFloat x, y, z, w;
    };

    /// \defgroup Vec4fOperators Vec4f operators.
    /// \brief Operators on Vec4f instances. See also the 
    /// \ref TemplateOperators "template operators" for more 
    /// operators automatically defined from the explicit ones
    /// defined here.
    /// \ingroup BasicTypes
    /// \{

				
    /// Print a Vec4f values to a ostream.
    inline ostream& operator<<( ostream &os, const Vec4f &v ) {
      os << v.x << " " << v.y << " " << v.z << " " << v.w;
      return os;
    } 
    /// Test two Vec4f for equality.
    inline bool operator==( const Vec4f &v1, const Vec4f &v2 ) {
      return v1.x == v2.x && v1.y == v2.y && v1.z == v2.z && v1.w == v2.w;
    }

    /// Addition between two Vec4f.
    inline Vec4f operator+( const Vec4f &v1, const Vec4f &v2 ) {
      return Vec4f( v1.x + v2.x, v1.y + v2.y, v1.z + v2.z, v1.w + v2.w );
    }
		
    /// Multiplication between Vec4f and float.
    inline Vec4f operator*( const Vec4f &v, const float &f ) {
      return Vec4f( v.x * f, v.y * f, v.z * f, v.w * f );
    }

    /// Multiplication between Vec4f and double.
    inline Vec4f operator*( const Vec4f &v, const double &d ) {
      return Vec4f( (H3DFloat)(v.x * d),
                    (H3DFloat)(v.y * d),
                    (H3DFloat)(v.z * d),
                    (H3DFloat)(v.w * d) );
    }

    /// Multiplication between Vec4f and int.
    inline Vec4f operator*( const Vec4f &v, const int &f ) {
      return Vec4f( v.x * f, v.y * f, v.z * f, v.w * f );
    }

    /// Multiplication between Vec4f and long.
    inline Vec4f operator*( const Vec4f &v, const long &f ) {
      return Vec4f( v.x * f, v.y * f, v.z * f, v.w * f );
    }

    /// Dot product between two Vec4f.
    inline H3DFloat operator*( const Vec4f &v1, const Vec4f &v2 ) {
      return v1.x*v2.x + v1.y*v2.y + v1.z*v2.z + v1.w*v2.w;
    }

    /// Multiplication with float.
    inline Vec4f operator*( const float &a, const Vec4f &b ) { return b * a; }

    /// Multiplication with double.
    inline Vec4f operator*( const double &a, const Vec4f &b ) { return b * a; }

    /// Multiplication with int.
    inline Vec4f operator*( const int &a, const Vec4f &b ) { return b * a; }

    /// Multiplication with long.
    inline Vec4f operator*( const long &a, const Vec4f &b ) { return b * a; }

    /// Unary minus.
    inline Vec4f operator-( const Vec4f &b ) { return b * (float)-1; }

    /// Subtraction between two Vec4f.
    inline Vec4f operator-( const Vec4f &a, const Vec4f &b ) { 
      return a + (-b); 
    }
				
    // \}
    
  }
}

#endif
