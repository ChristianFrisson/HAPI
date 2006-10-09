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
/// \file Vec2d.h
/// \brief Header file for Vec2d.
///
//
//////////////////////////////////////////////////////////////////////////////

#ifndef __VEC2D_H__
#define __VEC2D_H__

#include <HAPI.h>
#include <Types/H3DBasicTypes.h>
#include <H3DMath.h>
#include <Types/H3DTemplateOperators.h>
#include <Exception.h>
#include <Types/Vec2f.h>

namespace H3D {
  namespace ArithmeticTypes {
    /// A Vec2d specifies a high-precision 2d vector. The values of the 
    /// Vec2d are H3DDouble.
    /// \ingroup BasicTypes
    struct HAPI_API Vec2d {
      /// Exception thrown when error while normalizing vector types.
      H3D_API_EXCEPTION( Vec2dNormalizeError );
      
      /// Default constructor.
      Vec2d(): x(0.0), y(0.0) {}

      /// Constructor.
      Vec2d( H3DDouble _x,
             H3DDouble _y ) : x(_x), y(_y) {}
						
      /// Conversion from Vec2d.
      Vec2d( const Vec2f &v ): x( v.x ), y( v.y ) {}

      /// Returns the dot product between this vector and v. The 
      /// \ref Vec2dDotProduct "operator*" operator can also be used to 
      /// get the dot product between two vectors.
      inline H3DDouble dotProduct( const Vec2d &v ) const {
        return x*v.x + y*v.y;
      }
      
      /// Return the components by their index, 
      /// x has index 0
      /// y has index 1
      inline H3DDouble &operator[]( int i ) { 
        if( i == 0 ) return x;
        if( i == 1 ) return y;
        
        throw Exception::H3DAPIException( "Invalid index", 
                                          H3D_FULL_LOCATION );
      }

      /// Return the components by their index, 
      /// x has index 0
      /// y has index 1
      inline const H3DDouble &operator[]( int i ) const { 
        if( i == 0 ) return x;
        if( i == 1 ) return y;
        
        throw Exception::H3DAPIException( "Invalid index", 
                                          H3D_FULL_LOCATION );
      }

      /// Normalize the vector to be of length 1.
      /// \throws NormalizeError If the vector is of zero length.
      ///
      inline void normalize() {
        H3DDouble l2 = x*x+y*y;
        // if the length already is 1 we don't have to do anything
        if( H3DAbs(l2-1) > Constants::d_epsilon ) {
          H3DDouble l = H3DSqrt( l2 );
          if( H3DAbs(l) < Constants::d_epsilon ) {
            string s = "Trying to normalize zero length Vec2d.";
            throw Vec2dNormalizeError( s );
          } else {
            x /= l; 
            y /= l;
          }
        }
      }
      
      /// Normalize the vector to be of length 1.  If the vector is of 
      /// zero length nothing will be done.
      inline void normalizeSafe() {
        H3DDouble l2 = x*x+y*y;
        // if the length already is 1 we don't have to do anything
        if( H3DAbs(l2-1) > Constants::d_epsilon ) {
          H3DDouble l = H3DSqrt( l2 );
          if( H3DAbs(l) >= Constants::d_epsilon ) {
            x /= l; 
            y /= l;
          }
        }
      }

      /// Returns the length squeared of the vector.
      inline H3DDouble lengthSqr() {
        return x*x + y*y;
      }
      
      /// Returns the length of the vector.
      inline H3DDouble length() {
        return H3DSqrt( x*x + y*y );
      }
      
      /// The public values of the vector.
      H3DDouble x, y;
    };
    
    /// \defgroup Vec2dOperators Vec2d operators.
    /// \brief Operators on Vec2d instances. See also the 
    /// \ref TemplateOperators "template operators" for more 
    /// operators automatically defined from the explicit ones 
    /// defined here.
    /// \ingroup BasicTypes
    /// \{
				
    /// Function for printing a Vec2d to an ostream.
    inline ostream& operator<<( ostream &os, const Vec2d &v ) {
      os << v.x << " " << v.y;
      return os;
    } 

    /// Test two Vec2d for equality.
    inline bool operator==( const Vec2d &v1, const Vec2d &v2 ) {
      return v1.x == v2.x && v1.y == v2.y;
    }
    /// Addition between two Vec2d.
    inline Vec2d operator+( const Vec2d &v1, const Vec2d &v2 ) {
      return Vec2d( v1.x + v2.x, v1.y + v2.y );
    }
    /// Multiply a Vec2d with a double.
    inline Vec2d operator*( const Vec2d &v, const double &d ) {
      return Vec2d( v.x * d, v.y * d );
    }

    /// \anchor Vec2dDotProduct
    /// Get the dot product of two Vec2d.
    inline H3DDouble operator*( const Vec2d &v1, const Vec2d &v2 ) {
      return v1.dotProduct( v2 );
    }

    /// Multiplication with double.
    inline Vec2d operator*( const double &a, const Vec2d &b ) { return b * a; }
    
    /// Unary minus.
    inline Vec2d operator-( const Vec2d &b ) { return b * (double)-1; }

    /// Subtraction between two Vec2d.
    inline Vec2d operator-( const Vec2d &a, const Vec2d &b ) { return a + (-b); }
				
    // \}

  }
}

#endif
