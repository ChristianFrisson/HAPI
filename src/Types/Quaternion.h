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
/// \file Quaternion.h
/// \brief Header file for Quaternion.
///
//
//////////////////////////////////////////////////////////////////////////////

#ifndef __QUATERNION_H__
#define __QUATERNION_H__

#include "HAPI.h"
#include "H3DBasicTypes.h"
#include "H3DMath.h"
#include "Vec3f.h"

namespace H3D {
  namespace ArithmeticTypes {
    // forward declarations.
    class Matrix3f;
	class Matrix3d;
    class Rotation;

    /// Quaternion describes an arbitrary rotation.
    /// \ingroup BasicTypes
    class HAPI_API Quaternion {
    public:
      /// Default constructor.
      Quaternion(): v( 0, 0, 0 ), w( 0 ) {}

      /// Constructor. 
      Quaternion( H3DFloat x,
                  H3DFloat y,
                  H3DFloat z,
                  H3DFloat _w ) : v( x, y, z ), w(_w) {}
      
      /// Constructor.
      Quaternion( const Vec3f &_v, 
                  H3DFloat _w ) : v( _v ), w( _w ) {}
      
      /// Constructor. From Euler angles (yaw, pitch, roll ).
      explicit Quaternion( const Vec3f &euler_angles );

      /// Constructor. From Rotation object.
      Quaternion( const Rotation &r );
      
      /// Constructor. From Matrix3f that is a rotation matrix.  Assumes
      /// the matrix is orthogonal.
      explicit Quaternion( const Matrix3f &m );

      /// Constructor. From Matrix3d that is a rotation matrix.  Assumes
      /// the matrix is orthogonal.
      explicit Quaternion( const Matrix3d &m );

      /// Returns the Quaternion norm.
      inline H3DFloat norm() {
        return w*w + v.dotProduct( v );
      }

      /// Normalize the Quaternion, i.e. scale it so that the magnitude
      /// is 1.
      inline void normalize() {
        H3DFloat n = norm();
        if (H3DAbs( n ) > Constants::f_epsilon ) {
          H3DFloat length = H3DSqrt( n );
          v = v / length;
          w = w / length;
        }
      }
      
      /// dotProduct() returns the cos(angle) between two quaternions
      inline H3DFloat dotProduct( const Quaternion &q) const {
        return v.x*q.v.x + v.y*q.v.y + v.z*q.v.z + w*q.w;
      }
      
      /// Returns the conjugate of the Quaternion.
      inline Quaternion conjugate() {
        return Quaternion( -v, w );
      }

      /// Returns the inverse of the Quaternion.
      inline Quaternion inverse();
      
      /// Get the euler angles( yaw, pitch, roll ) representation of 
      /// the Quaternion.
      Vec3f toEulerAngles();

      /// Spherical linear interpolation between two Quaternions.
      /// \param q Ending Quaternion
      /// \param t Interpolation value between 0 and 1.
      Quaternion slerp( const Quaternion &q,
                        H3DFloat t ) const;

      /// The Quaternion vector part.
      Vec3f v;
      /// The Quaternion scalar part.
      H3DFloat w;
    };

    /// \defgroup QuaternionOperators Quaternion operators.
    /// \brief Operators on Quaternion instances. See also the 
    /// \ref TemplateOperators "template operators" for more operators
    /// automatically defined from the explicit ones defined here.
    /// \ingroup BasicTypes
    /// \{

    /// Function for printing a Quaternion to an ostream.
	  inline ostream& operator<<( ostream &os, const Quaternion &q ) {
		  os << q.v << " " << q.w;
		  return os;
	  }

    /// Equality between two Quaternion instances.
    inline bool operator==( const Quaternion &q1, const Quaternion &q2 ) {
      return q1.v == q2.v && q1.w == q2.w;
    }

    /// Multiplication of Quaternions. If q1 and q2 are unit quaternions, 
    /// then return value will also be a unit quaternion.
    inline Quaternion operator*( const Quaternion &q1, 
                                 const Quaternion &q2 ) {    
      return Quaternion( q1.w * q2.v + q2.w * q1.v + q1.v % q2.v,
                         q1.w * q2.w - q1.v * q2.v );
    }

    /// Multiplacation by a double.
    inline Quaternion operator*( const Quaternion &q, 
                                 double d ) {    
      return Quaternion( d *q.v, (H3DFloat)(d * q.w) );
    }

    /// Multiplacation by a float.
    inline Quaternion operator*( const Quaternion &q, 
                                 float d ) {    
      return Quaternion( d *q.v, d * q.w );
    }

    /// Multiplacation by an int.
    inline Quaternion operator*( const Quaternion &q, 
                                 int d ) {    
      return Quaternion( d *q.v, (H3DFloat) (d * q.w) );
    }

    /// Multiplacation by a long.
    inline Quaternion operator*( const Quaternion &q, 
                                 long d ) {    
      return Quaternion( d *q.v, (H3DFloat) (d * q.w) );
    }

    /// Multiplication with float.
    inline Quaternion operator*( const float &a, 
                                 const Quaternion &b ) { 
      return b * a;
    }

    /// Multiplication with double.
    inline Quaternion operator*( const double &a, 
                                 const Quaternion &b ) { 
      return b * a;
    }

    /// Multiplication with int.
    inline Quaternion operator*( const int &a, 
                                 const Quaternion &b ) { 
      return b * a;
    }

    /// Multiplication with long.
    inline Quaternion operator*( const long &a, 
                                 const Quaternion &b ) { 
      return b * a;
    }

    /// Addition of Quaternions. The result is not necessarily a unit 
    /// quaternion even if  and  are unit quaternion
    inline Quaternion operator+( const Quaternion &q1, 
                                 const Quaternion &q2 ) {
      return Quaternion( q1.v + q2.v,
                         q1.w + q2.w );
    }
    /// \}

    /// Unary minus.
    inline Quaternion operator-( const Quaternion &q ) { return q * -1; }
    
    /// Subtraction between two Quaternions. The result is not necessarily a unit 
    /// quaternion even if a and b are unit quaternion.
    inline Quaternion operator-( const Quaternion &a, const Quaternion &b ) { 
      return a + (-b); 
    }

    /// \}

    // Returns the inverse of the Quaternion.
    inline Quaternion Quaternion::inverse() {
      H3DFloat n = norm();
      if ( H3DAbs(n ) > Constants::f_epsilon ) {
        return Quaternion(0,0,0,0);
      } else {
        return conjugate() / n;
      }
    }
  }
}

#endif
