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
/// \file Vec4d.h
/// \brief Header file for Vec4d.
///
//
//////////////////////////////////////////////////////////////////////////////

#ifndef __VEC4D_H__
#define __VEC4D_H__

#include "HAPI.h"
#include "H3DBasicTypes.h"
#include "H3DTemplateOperators.h"
#include "Vec4f.h"
#include "Exception.h"

namespace H3D {
  namespace ArithmeticTypes {
    /// A Vec4d specifies a high-precision 4d vector. The values of the 
    /// Vec4d are H3DDouble.
    /// \ingroup BasicTypes
    struct HAPI_API Vec4d {
      /// Default constructor.
      Vec4d(): x(0), y(0), z(0), w(1){}
      
      /// Constructor.
      Vec4d( H3DDouble _x,
             H3DDouble _y,
             H3DDouble _z,
             H3DDouble _w = 1 ) : x(_x), y(_y), z(_z), w(_w) {}
      
      /// Conversion from Vec4d.
      Vec4d( const Vec4f &v ): x( v.x ), y( v.y ), z( v.z ), w( v.w ) {}

      /// Return the components by their index, 
      /// x has index 0
      /// y has index 1
      /// z has index 2
      /// w has index 3
      inline H3DDouble &operator[]( int i ) { 
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
      inline const H3DDouble &operator[]( int i ) const { 
        if( i == 0 ) return x;
        if( i == 1 ) return y;
        if( i == 2 ) return z;
        if( i == 3 ) return w;
        
        throw Exception::H3DAPIException( "Invalid index", 
                                          H3D_FULL_LOCATION );
      }

      /// The public values of the vector.
      H3DDouble x, y, z, w;
    };
    
    /// \defgroup Vec4dOperators Vec4d operators.
    /// \brief Operators on Vec4d instances. See also the 
    /// \ref TemplateOperators "template operators" for more 
    /// operators automatically defined from the explicit ones
    /// defined here.
    /// \ingroup BasicTypes
    /// \{

				
    /// Print a Vec4d values to a ostream.
    inline ostream& operator<<( ostream &os, const Vec4d &v ) {
      os << v.x << " " << v.y << " " << v.z << " " << v.w;
      return os;
    } 
    /// Test two Vec4d for equality.
    inline bool operator==( const Vec4d &v1, const Vec4d &v2 ) {
      return v1.x == v2.x && v1.y == v2.y && v1.z == v2.z && v1.w == v2.w;
    }

    /// Addition between two Vec4d.
    inline Vec4d operator+( const Vec4d &v1, const Vec4d &v2 ) {
      return Vec4d( v1.x + v2.x, v1.y + v2.y, v1.z + v2.z, v1.w + v2.w );
    }
		
    /// Multiplication between Vec4d and double.
    inline Vec4d operator*( const Vec4d &v, const double &d ) {
      return Vec4d( v.x * d, v.y * d, v.z * d, v.w * d );
    }

    /// Dot product between two Vec4d.
    inline H3DDouble operator*( const Vec4d &v1, const Vec4d &v2 ) {
      return v1.x*v2.x + v1.y*v2.y + v1.z*v2.z + v1.w*v2.w;
    }

    /// Multiplication with float.
    inline Vec4d operator*( const double &a, const Vec4d &b ) { return b * a; }

    /// Unary minus.
    inline Vec4d operator-( const Vec4d &b ) { return b * (double)-1; }

    /// Subtraction between two Vec4d.
    inline Vec4d operator-( const Vec4d &a, const Vec4d &b ) { return a + (-b); }
				
    /// \}
  }
}

#endif
