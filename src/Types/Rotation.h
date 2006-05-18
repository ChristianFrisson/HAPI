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
/// \file Rotation.h
/// \brief Header file for Rotation.
///
//
//////////////////////////////////////////////////////////////////////////////

#ifndef __ROTATION_H__
#define __ROTATION_H__

#include <HAPI.h>
#include <Types/H3DBasicTypes.h>
#include <H3DMath.h>
#include <Types/Vec3f.h>
#include <Types/H3DTemplateOperators.h>

namespace H3D {
  namespace ArithmeticTypes {
    // forward declarations.
    class Matrix3f;
    class Matrix3d;
    class Quaternion;

    /// Rotation describes an arbitrary rotation. It specifies an axis to
    /// rotate around and the angle to rotate.
    /// \ingroup BasicTypes
    class HAPI_API Rotation {
    public:
      /// Default constructor.
      Rotation() : axis( 0,0,0 ), angle( 0 ) {}

      /// Constructor. x, y, z is the vector to rotate around and a is the
      /// angle.
      Rotation( H3DFloat x,
                H3DFloat y,
                H3DFloat z,
                H3DFloat a ) : axis( x, y, z ), angle(a) {}

      /// Constructor.
      /// \param _axis The axis of the rotation.
      /// \param _angle The angle of the rotation.
      Rotation( const Vec3f &_axis, 
                H3DFloat _angle ) : axis( _axis ), angle( _angle ) {}

      /// Constructor.
      /// Constructs the shortest rotation the goes from n1 to n2.
      /// Both n1 and n2 must be unit vectors.
      Rotation( const Vec3f &n1, const Vec3f &n2 );

      /// Constructor. From Euler angles (yaw, pitch, roll ).
      explicit Rotation( const Vec3f &euler_angles );

      /// Constructor. From Quaternion object.
      Rotation( const Quaternion &r );
      
      /// Constructor. From Matrix3f that is a rotation matrix. Assumes
      /// the matrix is orthogonal.
      explicit Rotation( const Matrix3f &m );

      /// Constructor. From Matrix3d that is a rotation matrix. Assumes
      /// the matrix is orthogonal.
      explicit Rotation( const Matrix3d &m );
      
      /// Get the euler angles( yaw, pitch, roll ) representation of 
      /// the Rotation. 
      Vec3f toEulerAngles(); 

      /// Spherical linear interpolation between two Rotations.
      /// \param r Ending Rotation
      /// \param t Interpolation value between 0 and 1.
      Rotation slerp( const Rotation &r, 
                      H3DFloat t ) const;

      /// The axis the rotation is around.
      Vec3f axis;

      /// The angle of the rotation. 
      H3DFloat angle;
    };

    /// \defgroup RotationOperators Rotation operators.
    /// \brief Operators on Rotation instances. See also the 
    /// \ref TemplateOperators "template operators" for more operators
    /// automatically defined from the explicit ones defined here.
    /// \ingroup BasicTypes
    /// \{

    /// Test two Rotation instances for equality.
    inline bool operator==( const Rotation &r1, const Rotation &r2 ) {
      return r1.axis == r2.axis && r1.angle == r2.angle;
    }
		
    /// Negation of a Rotation is the Rotation around the same axis
    /// but in the other direction.
    ///
    inline Rotation operator-( const Rotation &r ) {
      return Rotation( r.axis, -r.angle );
    }

    /// Multiplacation by a double.
    inline Rotation operator*( const Rotation &r, 
                               double d ) {    
      return Rotation( r.axis, (H3DFloat) (r.angle * d) );
    }

    /// Multiplacation by a float.
    inline Rotation operator*( const Rotation &r, 
                               float f ) {    
      return Rotation( r.axis, (H3DFloat) (r.angle * f) );
    }

    /// Multiplacation by a int.
    inline Rotation operator*( const Rotation &r, 
                               int i ) {    
      return Rotation( r.axis, r.angle * i );
    }

    /// Multiplacation by a long.
    inline Rotation operator*( const Rotation &r, 
                               long i ) {    
      return Rotation( r.axis, r.angle * i );
    }

    /// Multiplication with float.
    inline Rotation operator*( const float &a, 
                               const Rotation &b ) { 
      return b * a;
    }

    /// Multiplication with double.
    inline Rotation operator*( const double &a, 
                               const Rotation &b ) { 
      return b * a;
    }

    /// Multiplication with int.
    inline Rotation operator*( const int &a, 
                               const Rotation &b ) { 
      return b * a;
    }

    /// Multiplication with long.
    inline Rotation operator*( const long &a, 
                               const Rotation &b ) { 
      return b * a;
    }

    /// Function for printing a Rotation to an ostream.
	  inline ostream& operator<<( ostream &os, const Rotation &r ) {
		  os << r.axis << " " << r.angle;
		  return os;
	  }

    /// Multiplication of Rotation is the composition of the rotations.
    Rotation HAPI_API operator*( const Rotation &r1 ,const Rotation &r2 );
   }
}

#endif
