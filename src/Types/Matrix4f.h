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
/// \file Matrix4f.h
/// \brief Header file for Matrix4f.
///
//
//////////////////////////////////////////////////////////////////////////////

#ifndef __MATRIX4F_H__
#define __MATRIX4F_H__

#include "HAPI.h"
#include "H3DBasicTypes.h"
#include "Vec4f.h"
#include "H3DTemplateOperators.h"
#include "Exception.h"
#include "Matrix3f.h"

namespace H3D {
  namespace ArithmeticTypes {
    class Quaternion;
    class Rotation;
    class Matrix4d;
    
    /// Representation of a 4x4 matrix.
    /// \ingroup BasicTypes
    class HAPI_API Matrix4f {
    public:
      /// Thrown when trying to perform operations on a singular matrix
      /// that are not allowed.
      H3D_API_EXCEPTION( SingularMatrix4f );

      /// Constructor. Creates an identity matrix.
      inline Matrix4f() { setToIdentity(); }
      
      /// Constructor.
      inline Matrix4f( H3DFloat m00, H3DFloat m01, 
                       H3DFloat m02, H3DFloat m03,
                       H3DFloat m10, H3DFloat m11, 
                       H3DFloat m12, H3DFloat m13,
                       H3DFloat m20, H3DFloat m21, 
                       H3DFloat m22, H3DFloat m23,
                       H3DFloat m30, H3DFloat m31, 
                       H3DFloat m32, H3DFloat m33 ) {
        m[0][0] = m00; m[0][1] = m01; m[0][2] = m02; m[0][3] = m03; 
        m[1][0] = m10; m[1][1] = m11; m[1][2] = m12; m[1][3] = m13; 
        m[2][0] = m20; m[2][1] = m21; m[2][2] = m22; m[2][3] = m23; 
        m[3][0] = m30; m[3][1] = m31; m[3][2] = m32; m[3][3] = m33; 
      }
				
      /// Constructor. From Rotation object.
      Matrix4f( const Rotation &r );

      /// Constructor. From Quaternion object.
      Matrix4f( const Quaternion &q );

      /// Constructor. From Matrix3f object. Creates a tranform matrix where
      /// the rotation part is the Matrix3f.
      explicit Matrix4f( const Matrix3f &m );

      /// Constructor. From Matrix4d object. 
      explicit Matrix4f( const Matrix4d &m );

      /// Set to the identity matrix.
      inline void setToIdentity() {
        m[0][0] = 1; m[0][1] = 0; m[0][2] = 0; m[0][3] = 0;
        m[1][0] = 0; m[1][1] = 1; m[1][2] = 0; m[1][3] = 0;
        m[2][0] = 0; m[2][1] = 0; m[2][2] = 1; m[2][3] = 0;
        m[3][0] = 0; m[3][1] = 0; m[3][2] = 0; m[3][3] = 1;
      }
			
      /// Returns the inverse of the matrix assuming that it is on 
      /// the form
      ///
      ///  [ m00 m01 m02 m03
      ///    m10 m11 m12 m13
      ///    m20 m21 m22 m23
      ///    0   0   0   1   ]
      ///
      Matrix4f transformInverse() const;
	
      /// Returns the transpose of the matrix.
      inline Matrix4f transpose() const {
        return Matrix4f( m[0][0], m[1][0], m[2][0], m[3][0],
												 m[0][1], m[1][1], m[2][1], m[3][1],
												 m[0][2], m[1][2], m[2][2], m[3][2],
                         m[0][3], m[1][3], m[2][3], m[3][3] );
      };

      /// Returns the inverse of the matrix.
      Matrix4f inverse() const;

      /// Get a row of the matrix.				
      inline H3DFloat* operator[]( const int i ) { return m[i]; }
				
      /// Get a row of the matrix.
      inline const H3DFloat* operator[]( const int i ) const { return m[i]; }

      /// Get a row of the matrix.
      inline Vec4f getRow( int i ) const { 
        return Vec4f( m[i][0], m[i][1], m[i][2], m[i][3] ); 
      }

      /// Get a column of the matrix.
      inline Vec4f getColumn( int i ) const { 
        return Vec4f( m[0][i], m[1][i], m[2][i], m[3][i] ); 
      }

			/// Returns the scale and rotation part of the Matrix4f.
			inline Matrix3f getScaleRotationPart() const {
				return Matrix3f( m[0][0], m[0][1], m[0][2],
												 m[1][0], m[1][1], m[1][2],
												 m[2][0], m[2][1], m[2][2] );
			}

			/// Returns the rotation part of the Matrix4f.
			Matrix3f getRotationPart() const;

    private:
      /// The matrix data.
      H3DFloat m[4][4];
    };

    /// \defgroup Matrix4fOperators Matrix4f operators.
    /// \brief Operators on Matrix4f instances. See also the 
    /// \ref TemplateOperators "template operators" for more operators
    /// automatically defined from the explicit ones defined here.
    /// \ingroup BasicTypes
    /// \{

    /// Multiplication between two Matrix4f instances.
    inline Matrix4f operator*( const Matrix4f &m1, const Matrix4f &m2 ) {
      return Matrix4f( 
  m1[0][0]*m2[0][0] + m1[0][1]*m2[1][0] + m1[0][2]*m2[2][0] + m1[0][3]*m2[3][0],
  m1[0][0]*m2[0][1] + m1[0][1]*m2[1][1] + m1[0][2]*m2[2][1] + m1[0][3]*m2[3][1],
  m1[0][0]*m2[0][2] + m1[0][1]*m2[1][2] + m1[0][2]*m2[2][2] + m1[0][3]*m2[3][2],
  m1[0][0]*m2[0][3] + m1[0][1]*m2[1][3] + m1[0][2]*m2[2][3] + m1[0][3]*m2[3][3],
  
  m1[1][0]*m2[0][0] + m1[1][1]*m2[1][0] + m1[1][2]*m2[2][0] + m1[1][3]*m2[3][0],
  m1[1][0]*m2[0][1] + m1[1][1]*m2[1][1] + m1[1][2]*m2[2][1] + m1[1][3]*m2[3][1],
  m1[1][0]*m2[0][2] + m1[1][1]*m2[1][2] + m1[1][2]*m2[2][2] + m1[1][3]*m2[3][2],
  m1[1][0]*m2[0][3] + m1[1][1]*m2[1][3] + m1[1][2]*m2[2][3] + m1[1][3]*m2[3][3],
  
  m1[2][0]*m2[0][0] + m1[2][1]*m2[1][0] + m1[2][2]*m2[2][0] + m1[2][3]*m2[3][0],
  m1[2][0]*m2[0][1] + m1[2][1]*m2[1][1] + m1[2][2]*m2[2][1] + m1[2][3]*m2[3][1],
  m1[2][0]*m2[0][2] + m1[2][1]*m2[1][2] + m1[2][2]*m2[2][2] + m1[2][3]*m2[3][2],
  m1[2][0]*m2[0][3] + m1[2][1]*m2[1][3] + m1[2][2]*m2[2][3] + m1[2][3]*m2[3][3],
  
  m1[3][0]*m2[0][0] + m1[3][1]*m2[1][0] + m1[3][2]*m2[2][0] + m1[3][3]*m2[3][0],
  m1[3][0]*m2[0][1] + m1[3][1]*m2[1][1] + m1[3][2]*m2[2][1] + m1[3][3]*m2[3][1],
  m1[3][0]*m2[0][2] + m1[3][1]*m2[1][2] + m1[3][2]*m2[2][2] + m1[3][3]*m2[3][2],
  m1[3][0]*m2[0][3] + m1[3][1]*m2[1][3] + m1[3][2]*m2[2][3] + m1[3][3]*m2[3][3]
  );
    }

    /// Addition between two Matrix4f instances.
    inline Matrix4f operator+( const Matrix4f &m1, const Matrix4f &m2 ) {
      return Matrix4f( 
  m1[0][0]+m2[0][0], m1[0][1]+m2[0][1], m1[0][2]+m2[0][2], m1[0][3]+m2[0][3],
  m1[1][0]+m2[1][0], m1[1][1]+m2[1][1], m1[1][2]+m2[1][2], m1[1][3]+m2[1][3],
  m1[2][0]+m2[2][0], m1[2][1]+m2[2][1], m1[2][2]+m2[2][2], m1[2][3]+m2[2][3],
  m1[3][0]+m2[3][0], m1[3][1]+m2[3][1], m1[3][2]+m2[3][2], m1[3][3]+m2[3][3] );
    }
    
    /// Muliplication between Matrix4f and float.
    inline Matrix4f operator*( const Matrix4f &m, const float &f ) {
      return Matrix4f( m[0][0]*f, m[0][1]*f, m[0][2]*f, m[0][3]*f,
                       m[1][0]*f, m[1][1]*f, m[1][2]*f, m[1][3]*f,
                       m[2][0]*f, m[2][1]*f, m[2][2]*f, m[2][3]*f,
                       m[3][0]*f, m[3][1]*f, m[3][2]*f, m[3][3]*f );
    }
    /// Muliplication between Matrix4f and double.
    inline Matrix4f operator*( const Matrix4f &m, const double &d ) {
      return Matrix4f( (H3DFloat)(m[0][0]*d), (H3DFloat)(m[0][1]*d), (H3DFloat)(m[0][2]*d), (H3DFloat)(m[0][3]*d),
                       (H3DFloat)(m[1][0]*d), (H3DFloat)(m[1][1]*d), (H3DFloat)(m[1][2]*d), (H3DFloat)(m[1][3]*d),
                       (H3DFloat)(m[2][0]*d), (H3DFloat)(m[2][1]*d), (H3DFloat)(m[2][2]*d), (H3DFloat)(m[2][3]*d),
                       (H3DFloat)(m[3][0]*d), (H3DFloat)(m[3][1]*d), (H3DFloat)(m[3][2]*d), (H3DFloat)(m[3][3]*d) );
    }

    /// Muliplication between Matrix4f and int.
    inline Matrix4f operator*( const Matrix4f &m, const int &f ) {
      return Matrix4f( m[0][0]*f, m[0][1]*f, m[0][2]*f, m[0][3]*f,
                       m[1][0]*f, m[1][1]*f, m[1][2]*f, m[1][3]*f,
                       m[2][0]*f, m[2][1]*f, m[2][2]*f, m[2][3]*f,
                       m[3][0]*f, m[3][1]*f, m[3][2]*f, m[3][3]*f );
    }

    /// Muliplication between Matrix4f and long.
    inline Matrix4f operator*( const Matrix4f &m, const long &f ) {
      return Matrix4f( m[0][0]*f, m[0][1]*f, m[0][2]*f, m[0][3]*f,
                       m[1][0]*f, m[1][1]*f, m[1][2]*f, m[1][3]*f,
                       m[2][0]*f, m[2][1]*f, m[2][2]*f, m[2][3]*f,
                       m[3][0]*f, m[3][1]*f, m[3][2]*f, m[3][3]*f );
    }
	  
    /// Equality between two Matrix4f instances.
    inline bool operator==( const Matrix4f &m1, const Matrix4f &m2 ) {
       return m1[0][0]==m2[0][0] && m1[0][1]==m2[0][1] && m1[0][2]==m2[0][2] && 
   m1[0][3]==m2[0][3] && m1[1][0]==m2[1][0] && m1[1][1]==m2[1][1] && 
   m1[1][2]==m2[1][2] && m1[1][3]==m2[1][3] && m1[2][0]==m2[2][0] && 
   m1[2][1]==m2[2][1] && m1[2][2]==m2[2][2] && m1[2][3]==m2[2][3] &&
   m1[3][0]==m2[3][0] && m1[3][1]==m2[3][1] && m1[3][2]==m2[3][2] && 
   m1[3][3]==m2[3][3];
     }

    /// Function for printing a Matrix4f to an ostream.
    HAPI_API ostream& operator<<( ostream &os, const Matrix4f &m );

    /// Multiplication with float.
    inline Matrix4f operator*( const float &a, const Matrix4f &b ) { return b * a; }

    /// Multiplication with double.
    inline Matrix4f operator*( const double &a, const Matrix4f &b ) { return b * a; }

    /// Multiplication with int.
    inline Matrix4f operator*( const int &a, const Matrix4f &b ) { return b * a; }

    /// Multiplication with long.
    inline Matrix4f operator*( const long &a, const Matrix4f &b ) { return b * a; }
    
    /// Unary minus.
    inline Matrix4f operator-( const Matrix4f &m ) { return m * -1; }
    
    /// Subtraction between two Matrix4f.
    inline Matrix4f operator-( const Matrix4f &a, const Matrix4f &b ) 
        { return a + (-b); }
    
    /// \}
  }
}

#endif
