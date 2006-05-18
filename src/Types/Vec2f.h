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
/// \file Vec2f.h
/// \brief Header file for Vec2f.
///
//
//////////////////////////////////////////////////////////////////////////////

#ifndef __VEC2F_H__
#define __VEC2F_H__

#include "HAPI.h"
#include "H3DBasicTypes.h"
#include "H3DMath.h"
#include "H3DTemplateOperators.h"
#include "Exception.h"

namespace H3D {
  namespace ArithmeticTypes {
    struct Vec2d;

    /// A Vec2f specifies a 2d vector. The values of the Vec2f are
    /// H3DFloat.
    /// \ingroup BasicTypes
    struct HAPI_API Vec2f {
      /// Exception thrown when error while normalizing vector types.
      H3D_API_EXCEPTION( Vec2fNormalizeError );

      /// Default constructor.
      Vec2f(): x(0), y(0) {}

      /// Constructor.
      Vec2f( H3DFloat _x,
             H3DFloat _y ) : x(_x), y(_y) {}
						
      /// Conversion from Vec2d.
      explicit Vec2f( const Vec2d &v );

      /// Returns the dot product between this vector and v. The 
      /// \ref Vec2fDotProduct "operator*" operator can also be used to 
      /// get the dot product between two vectors.
      inline H3DFloat dotProduct( const Vec2f &v ) const {
        return x*v.x + y*v.y;
      }
						
      /// Normalize the vector to be of length 1.
      /// \throws NormalizeError If the vector is of zero length.
      ///
      inline void normalize() {
        H3DFloat l2 = x*x+y*y;
        // if the length already is 1 we don't have to do anything
        if( H3DAbs(l2-1) > Constants::f_epsilon ) {
          H3DFloat l = H3DSqrt( l2 );
          if( H3DAbs(l) < Constants::f_epsilon ) {
            string s = "Trying to normalize zero length Vec2f.";
            throw Vec2fNormalizeError( s );
          } else {
            x /= l; 
            y /= l;
          }
        }
      }

      /// Normalize the vector to be of length 1.  If the vector is of 
      /// zero length nothing will be done.
      inline void normalizeSafe() {
        H3DFloat l2 = x*x+y*y;
        // if the length already is 1 we don't have to do anything
        if( H3DAbs(l2-1) > Constants::f_epsilon ) {
          H3DFloat l = H3DSqrt( l2 );
          if( H3DAbs(l) >= Constants::f_epsilon ) {
            x /= l; 
            y /= l;
          }
        }
      }

      /// Returns the length squeared of the vector.
      inline H3DFloat lengthSqr() {
        return x*x + y*y;
      }

      /// Returns the length of the vector.
      inline H3DFloat length() {
        return H3DSqrt( x*x + y*y );
      }

      /// The public values of the vector.
      H3DFloat x, y;
    };
    
    /// \defgroup Vec2fOperators Vec2f operators.
    /// \brief Operators on Vec2f instances. See also the 
    /// \ref TemplateOperators "template operators" for more 
    /// operators automatically defined from the explicit ones 
    /// defined here.
    /// \ingroup BasicTypes
    /// \{
				
    /// Function for printing a Vec2f to an ostream.
    inline ostream& operator<<( ostream &os, const Vec2f &v ) {
      os << v.x << " " << v.y;
      return os;
    } 

    /// Test two Vec2f for equality.
    inline bool operator==( const Vec2f &v1, const Vec2f &v2 ) {
      return v1.x == v2.x && v1.y == v2.y;
    }
    /// Addition between two Vec2f.
    inline Vec2f operator+( const Vec2f &v1, const Vec2f &v2 ) {
      return Vec2f( v1.x + v2.x, v1.y + v2.y );
    }
    /// Multiply a Vec2f with a float.
    inline Vec2f operator*( const Vec2f &v, const float &f ) {
      return Vec2f( v.x * f, v.y * f );
    }

    /// Multiply a Vec2f with a double.
    inline Vec2f operator*( const Vec2f &v, const double &d ) {
      return Vec2f( (H3DFloat)(v.x * d), 
                    (H3DFloat)(v.y * d) );
    }

    /// Multiply a Vec2f with an int.
    inline Vec2f operator*( const Vec2f &v, const int &i ) {
      return Vec2f( v.x * i, v.y * i );
    }

    /// Multiply a Vec2f with an long.
    inline Vec2f operator*( const Vec2f &v, const long &i ) {
      return Vec2f( v.x * i, v.y * i );
    }

    /// \anchor Vec2fDotProduct
    /// Get the dot product of two Vec2f.
    inline H3DFloat operator*( const Vec2f &v1, const Vec2f &v2 ) {
      return v1.dotProduct( v2 );
    }

    /// Multiplication with float.
    inline Vec2f operator*( const float &a, const Vec2f &b ) { return b * a; }

    /// Multiplication with double.
    inline Vec2f operator*( const double &a, const Vec2f &b ) { return b * a; }

    /// Multiplication with int.
    inline Vec2f operator*( const int &a, const Vec2f &b ) { return b * a; }

    /// Multiplication with long.
    inline Vec2f operator*( const long &a, const Vec2f &b ) { return b * a; }

    /// Unary minus.
    inline Vec2f operator-( const Vec2f &b ) { return b * (float)-1; }

    /// Subtraction between two Vec2f.
    inline Vec2f operator-( const Vec2f &a, const Vec2f &b ) { return a + (-b); }
    
    // \}
  }
}
#endif
