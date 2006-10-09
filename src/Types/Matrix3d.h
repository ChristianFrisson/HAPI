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
/// \file Matrix3d.h
/// \brief Header file for Matrix3d.
///
//
//////////////////////////////////////////////////////////////////////////////

#ifndef __MATRIX3D_H__
#define __MATRIX3D_H__

#include "HAPI.h"
#include "H3DBasicTypes.h"
#include "Vec3d.h"
#include "Exception.h"

namespace H3D {
  namespace ArithmeticTypes {
    /// forward declarations.
    class Quaternion;
    class Rotation;
	class Matrix3f;

    /// Representation of a 3x3 matrix.
    /// \ingroup BasicTypes
    class HAPI_API Matrix3d {
    public:
      /// Thrown when trying to perform operations on a singular matrix
      /// that are not allowed.
      H3D_API_EXCEPTION( SingularMatrix3d );

      /// Constructor. Creates an identity matrix.
      inline Matrix3d() {
        setToIdentity();
      }

      /// Constructor.
      inline Matrix3d( H3DDouble m00, H3DDouble m01, H3DDouble m02,
                       H3DDouble m10, H3DDouble m11, H3DDouble m12,
                       H3DDouble m20, H3DDouble m21, H3DDouble m22 ) {
        m[0][0] = m00; m[0][1] = m01; m[0][2] = m02; 
        m[1][0] = m10; m[1][1] = m11; m[1][2] = m12; 
        m[2][0] = m20; m[2][1] = m21; m[2][2] = m22; 
      }

      /// Conversion from Matrix3f.
      Matrix3d( const Matrix3f &matrix );

      /// Constructor. From Rotation object.
      Matrix3d( const Rotation &r );

      /// Constructor. From Quaternion object.
      Matrix3d( const Quaternion &q );

      /// Set the Matrix3d to the identity matrix.
      inline void setToIdentity() {
        m[0][0] = 1; m[0][1] = 0; m[0][2] = 0;
        m[1][0] = 0; m[1][1] = 1; m[1][2] = 0;
        m[2][0] = 0; m[2][1] = 0; m[2][2] = 1;
      }
				
      /// Returns the transpose of the matrix.
      inline Matrix3d transpose() const {
        return Matrix3d( m[0][0], m[1][0], m[2][0],
                         m[0][1], m[1][1], m[2][1],
                         m[0][2], m[1][2], m[2][2] );
      };

      /// Returns the inverse of the matrix.
      Matrix3d inverse() const;

      /// Get a row of the matrix.
      inline H3DDouble* operator[]( int i ) { return m[i]; }

      /// Get a row of the matrix.
      inline const H3DDouble* operator[]( int i ) const { return m[i]; }

      /// Get a row of the matrix.
      inline Vec3d getRow( int i ) const { 
        return Vec3d( m[i][0], m[i][1], m[i][2] ); 
      }

      /// Get an element in the matrix.
      inline H3DDouble getElement( int i, int j ) const { 
        return m[i][j];
      }

      /// Set an element in the matrix.
      inline void setElement( int i, int j, H3DDouble v ) { 
        m[i][j] = v;
      }

      /// Get the euler angles( yaw, pitch, roll ) representation of 
      /// the rotation matrix. The Matrix3d must be a rotation matrix.
      Vec3d toEulerAngles();

      /// Get a column of the matrix.
      inline Vec3d getColumn( int i ) const { 
        return Vec3d( m[0][i], m[1][i], m[2][i] ); 
      }

      /// Get the scaling part of the matrix for each axis.
      Vec3d getScalePart() const;

    private:
      /// The matrix data.
      H3DDouble m[3][3];
    };
    
    /// \defgroup Matrix3dOperators Matrix3d operators.
    /// \brief Operators on Matrix3d instances. See also the 
    /// \ref TemplateOperators "template operators" for more operators
    /// automatically defined from the explicit ones defined here.
    /// \ingroup BasicTypes
    /// \{
				
    /// Multiplication between two Matrix3d instances.
    inline Matrix3d operator*( const Matrix3d &m1, const Matrix3d &m2 ) {
      return Matrix3d( 
                      m1[0][0]*m2[0][0] + m1[0][1]*m2[1][0] + m1[0][2]*m2[2][0],
                      m1[0][0]*m2[0][1] + m1[0][1]*m2[1][1] + m1[0][2]*m2[2][1],
                      m1[0][0]*m2[0][2] + m1[0][1]*m2[1][2] + m1[0][2]*m2[2][2],
                      m1[1][0]*m2[0][0] + m1[1][1]*m2[1][0] + m1[1][2]*m2[2][0],
                      m1[1][0]*m2[0][1] + m1[1][1]*m2[1][1] + m1[1][2]*m2[2][1],
                      m1[1][0]*m2[0][2] + m1[1][1]*m2[1][2] + m1[1][2]*m2[2][2],
                      m1[2][0]*m2[0][0] + m1[2][1]*m2[1][0] + m1[2][2]*m2[2][0],
                      m1[2][0]*m2[0][1] + m1[2][1]*m2[1][1] + m1[2][2]*m2[2][1],
                      m1[2][0]*m2[0][2] + m1[2][1]*m2[1][2] + m1[2][2]*m2[2][2] );
    }
    
    /// Addition between two Matrix3d instances.
    inline Matrix3d operator+( const Matrix3d &m1, const Matrix3d &m2 ) {
      return Matrix3d( 
                      m1[0][0]+m2[0][0], m1[0][1]+m2[0][1], m1[0][2]+m2[0][2],
                      m1[1][0]+m2[1][0], m1[1][1]+m2[1][1], m1[1][2]+m2[1][2],
                      m1[2][0]+m2[2][0], m1[2][1]+m2[2][1], m1[2][2]+m2[2][2] );
    }
		
    /// Muliplication between Matrix3d and float. 
    inline Matrix3d operator*( const Matrix3d &m, const float &f ) {
      return Matrix3d( m[0][0]*f, m[0][1]*f, m[0][2]*f,
                       m[1][0]*f, m[1][1]*f, m[1][2]*f,
                       m[2][0]*f, m[2][1]*f, m[2][2]*f );
    }

    /// Muliplication between Matrix3d and double. 
    inline Matrix3d operator*( const Matrix3d &m, const double &d ) {
      return Matrix3d( 
   (H3DDouble)(m[0][0]*d), (H3DDouble)(m[0][1]*d), (H3DDouble)(m[0][2]*d),
   (H3DDouble)(m[1][0]*d), (H3DDouble)(m[1][1]*d), (H3DDouble)(m[1][2]*d),
   (H3DDouble)(m[2][0]*d), (H3DDouble)(m[2][1]*d), (H3DDouble)(m[2][2]*d ));
    }

    /// Muliplication between Matrix3d and int. 
    inline Matrix3d operator*( const Matrix3d &m, const int &f ) {
      return Matrix3d( m[0][0]*f, m[0][1]*f, m[0][2]*f,
                       m[1][0]*f, m[1][1]*f, m[1][2]*f,
                       m[2][0]*f, m[2][1]*f, m[2][2]*f );
    }

    /// Muliplication between Matrix3d and long. 
    inline Matrix3d operator*( const Matrix3d &m, const long &f ) {
      return Matrix3d( m[0][0]*f, m[0][1]*f, m[0][2]*f,
                       m[1][0]*f, m[1][1]*f, m[1][2]*f,
                       m[2][0]*f, m[2][1]*f, m[2][2]*f );
    }

    /// Equality between two Matrix3d instances.
    inline bool operator==( const Matrix3d &m1, const Matrix3d &m2 ) {
      return (
         m1[0][0]==m2[0][0] && m1[0][1]==m2[0][1] && m1[0][2]==m2[0][2] &&
         m1[1][0]==m2[1][0] && m1[1][1]==m2[1][1] && m1[1][2]==m2[1][2] && 
         m1[2][0]==m2[2][0] && m1[2][1]==m2[2][1] && m1[2][2]==m2[2][2] 
         );
     }

    /// Function for printing a Matrix3d to an ostream.
	  HAPI_API ostream& operator<<( ostream &os, const Matrix3d &m );

    /// Multiplication with float.
    inline Matrix3d operator*( const float &a, const Matrix3d &b ) { return b * a; }

    /// Multiplication with double.
    inline Matrix3d operator*( const double &a, const Matrix3d &b ) { return b * a; }

    /// Multiplication with int.
    inline Matrix3d operator*( const int &a, const Matrix3d &b ) { return b * a; }

    /// Multiplication with long.
    inline Matrix3d operator*( const long &a, const Matrix3d &b ) { return b * a; }
    
    /// Unary minus.
    inline Matrix3d operator-( const Matrix3d &m ) { return m * -1; }
    
    /// Subtraction between two Matrix3d.
    inline Matrix3d operator-( const Matrix3d &a, const Matrix3d &b ) 
        { return a + (-b); }

    /// \}
  }
}

#endif
