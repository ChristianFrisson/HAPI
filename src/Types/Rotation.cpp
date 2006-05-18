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
/// \file Rotation.cpp
/// \brief .cpp file for Rotation.
///
//
//////////////////////////////////////////////////////////////////////////////

#include "Rotation.h"
#include "Matrix3f.h"
#include "Quaternion.h"

using namespace H3D;
using namespace ArithmeticTypes;

Rotation H3D::ArithmeticTypes::operator*( const Rotation &r1 ,const Rotation &r2 ) {
  return (Quaternion) r1 * (Quaternion) r2;
}
/// Constructor. From Euler angles (yaw, pitch, roll ).
Rotation::Rotation( const Vec3f &euler_angles ) {
  *this = Quaternion( euler_angles );
}
 

/// Constructor. From Quaternion object.
Rotation::Rotation( const Quaternion &q ) {
  H3DFloat v2 = q.v * q.v;
  if( v2 < Constants::f_epsilon ) { 
    axis = Vec3f(1,0,0);
    angle = 0;
  } else {
    axis = q.v / H3DSqrt(v2);
	  if( q.w > 1 ) 
      angle = 0;
	  else if ( q.w < -1 ) 
      angle = (H3DFloat) Constants::pi;
    else
      angle = 2 * H3DAcos( q.w );
  }
}
      
/// Constructor. From Matrix3f that is a rotation matrix. 
Rotation::Rotation( const Matrix3f &m ) {
  Quaternion q(m);
  *this = Rotation::Rotation( q );
}

/// Constructor. From Matrix3f that is a rotation matrix. 
Rotation::Rotation( const Matrix3d &m ) {
  Quaternion q(m);
  *this = Rotation::Rotation( q );
}

/// Get the euler angles( yaw, pitch, roll ) representation of 
/// the Rotation. 
Vec3f Rotation::toEulerAngles() {
  return Matrix3f( *this ).toEulerAngles();
}

Rotation Rotation::slerp( const Rotation &r, 
                          H3DFloat t ) const {
  Quaternion q = *this;
  return q.slerp( r, t );
}

Rotation::Rotation( const Vec3f &n1, const Vec3f &n2 ) {
  H3DFloat dot_product = n1 * n2;

  bool use_perpendicular_axis = false;
  if( dot_product < -1.0 + Constants::f_epsilon ) {
    angle = (H3DFloat)Constants::pi;
    use_perpendicular_axis = true;
  } else if( dot_product > 1.0 - Constants::f_epsilon ) {
    angle = 0;
    use_perpendicular_axis = true;
  } else {
    angle = H3DAcos( dot_product );
  }

  if( use_perpendicular_axis ) {
    if( H3DAbs( n1.x ) > 0.5 ) axis = Vec3f( -n1.z, 0, n1.x );
    else if( H3DAbs( n1.y ) > 0.5 ) axis = Vec3f( n1.y, -n1.x, 0 );
    else axis = Vec3f( 0, n1.z, -n1.y );
  } else {
    axis = n1 % n2;
		if( axis * axis < Constants::f_epsilon )
			axis = Vec3f( 1, 0, 0 );
  }

  axis.normalize();
}
