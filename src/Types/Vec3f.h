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
/// \file Vec3f.h
/// \brief Header file for Vec3f.
///
//
//////////////////////////////////////////////////////////////////////////////

#ifndef __VEC3F_H__
#define __VEC3F_H__

#include "HAPI.h"
#include "H3DMath.h"
#include "H3DBasicTypes.h"
#include "H3DTemplateOperators.h"
#include "Exception.h"

namespace H3D {
  namespace ArithmeticTypes {
    struct Vec3d;

    /// A Vec3f specifies a high-precision 3d vector. The values of the 
    /// Vec3f are H3DFloat.		
    /// \ingroup BasicTypes
    struct HAPI_API Vec3f {

      /// Exception thrown when error while normalizing vector types.
      H3D_API_EXCEPTION( Vec3fNormalizeError );

      /// Default constructor.
      Vec3f(): x(0), y(0), z(0) {}

      /// Constructor.
      Vec3f( H3DFloat _x,
             H3DFloat _y,
             H3DFloat _z ) : x(_x), y(_y), z(_z) {}

      /// Conversion from Vec3d.
      explicit Vec3f( const Vec3d &v );

      /// Returns the dot product between this Vec3f and v. The 
      ///  \ref Vec3fDotProduct "operator*" operator can also be used
      /// to get the dot product between two vectors.
      inline H3DFloat dotProduct( const Vec3f &v ) const {
        return x*v.x + y*v.y + z*v.z;
      }

      /// Returns the cross product between this Vec3f and v. The 
      /// \ref Vec3fCrossProduct "operator*" operator can also be used
      /// to get the dot product between two vectors.
      inline Vec3f crossProduct( const Vec3f &v ) const {
        return 	Vec3f( y*v.z - z*v.y,
                       z*v.x - x*v.z,
                       x*v.y - y*v.x );
      }
				
      /// Normalize the vector to be of length 1.
      /// \throws NormalizeError If the vector is of zero length.
      ///
      inline void normalize() {
        H3DFloat l2 = x*x+y*y+z*z;
        // if the length already is 1 we don't have to do anything
        if( H3DAbs(l2-1) > Constants::f_epsilon ) {
          H3DFloat l = H3DSqrt( l2 );
          if( H3DAbs(l) < Constants::f_epsilon ) {
            string s = "Trying to normalize zero length Vec3f.";
            throw Vec3fNormalizeError( s );
          } else {
            x /= l; 
            y /= l;
            z /= l;
          }
        }
      }

      /// Normalize the vector to be of length 1. If the vector is of 
      /// zero length nothing will be done.
      inline void normalizeSafe() {
        H3DFloat l2 = x*x+y*y+z*z;
        // if the length already is 1 we don't have to do anything
        if( H3DAbs(l2-1) > Constants::f_epsilon ) {
          H3DFloat l = H3DSqrt( l2 );
          if( H3DAbs(l) >= Constants::f_epsilon ) {
            x /= l; 
            y /= l;
            z /= l;
          }
        }
      }
      
      /// Returns the length squeared of the vector.
      inline H3DFloat lengthSqr() {
        return x*x + y*y + z*z;
      }
		
      /// Returns the length of the vector.
      inline H3DFloat length() {
        return H3DSqrt( x*x + y*y + z*z );
      }
		
      inline H3DFloat &operator[]( int i ) { 
        if( i == 0 ) return x;
        if( i == 1 ) return y;
        if( i == 2 ) return z;
        
        throw Exception::H3DAPIException( "Invalid index", 
                                          H3D_FULL_LOCATION );
      }

      inline const H3DFloat &operator[]( int i ) const { 
        if( i == 0 ) return x;
        if( i == 1 ) return y;
        if( i == 2 ) return z;
        
        throw Exception::H3DAPIException( "Invalid index", 
                                          H3D_FULL_LOCATION );
      }

      /// The public values of the vector.
      H3DFloat x, y, z;
    };
    
    /// \defgroup Vec3fOperators Vec3f operators.
    /// \brief Operators on Vec3f instances. See also the 
    /// \ref TemplateOperators "template operators" for more 
    /// operators automatically defined from the explicit ones
    /// defined here.
    /// \ingroup BasicTypes
    /// \{
    
    /// Test two Vec3f for equality.
    inline bool operator==( const Vec3f &v1, const Vec3f &v2 ) {
      return v1.x == v2.x && v1.y == v2.y && v1.z == v2.z;
    }
    /// Addition between two Vec3f.
    inline Vec3f operator+( const Vec3f &v1, const Vec3f &v2 ) {
      return Vec3f( v1.x + v2.x, v1.y + v2.y, v1.z + v2.z );
    }

    /// Multiply a Vec3f with an int.
    inline Vec3f operator*( const Vec3f &v, const int &i ) {
      return Vec3f( v.x * i, v.y * i, v.z * i );
    }

    /// Multiply a Vec3f with a long.
    inline Vec3f operator*( const Vec3f &v, const long &i ) {
      return Vec3f( v.x * i, v.y * i, v.z * i );
    }

    /// Multiply a Vec3f with a long.
    inline Vec3f operator*( const Vec3f &v, const unsigned long &i ) {
      return Vec3f( v.x * i, v.y * i, v.z * i );
    }

    /// Multiply a Vec3f with a float.
    inline Vec3f operator*( const Vec3f &v, const float &f ) {
      return Vec3f( v.x * f, v.y * f, v.z * f );
    }

    /// Multiply a Vec3f with a double.
    inline Vec3f operator*( const Vec3f &v, const double &d ) {
      return Vec3f( (H3DFloat)(v.x * d),
                    (H3DFloat)(v.y * d), 
                    (H3DFloat)(v.z * d) );
    }

    /// \anchor Vec3fDotProduct
    /// Returns the dot product of two Vec3f.
    inline H3DFloat operator*( const Vec3f &v1, const Vec3f &v2 ) {
      return v1.dotProduct( v2 );
    }

    /// \anchor Vec3fCrossProduct
    /// Returns the dot product of two Vec3f.
    inline Vec3f operator%( const Vec3f &v1, const Vec3f &v2 ) {
      return v1.crossProduct( v2 );
    }
    
    /// Function for printing a Vec3f to an ostream.
	  inline ostream& operator<<( ostream &os, const Vec3f &v ) {
		  os << v.x << " " << v.y << " " << v.z;
		  return os;
	  }

    /// Multiplication with float.
    inline Vec3f operator*( const float &a, const Vec3f &b ) { return b * a; }

    /// Multiplication with double.
    inline Vec3f operator*( const double &a, const Vec3f &b ) { return b * a; }

    /// Multiplication with int.
    inline Vec3f operator*( const int &a, const Vec3f &b ) { return b * a; }

    /// Multiplication with long.
    inline Vec3f operator*( const long &a, const Vec3f &b ) { return b * a; }

    /// Multiplication with unsigned long.
    inline Vec3f operator*( const unsigned long &a, const Vec3f &b ) { 
      return b * a; }

    /// Unary minus.
    inline Vec3f operator-( const Vec3f &b ) { return b * (H3DFloat)-1; }

    /// Subtraction between two Vec3f.
    inline Vec3f operator-( const Vec3f &a, const Vec3f &b ) { return a + (-b); }
				
    /// \}

  }
}

#endif
