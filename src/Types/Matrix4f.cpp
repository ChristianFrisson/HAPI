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
/// \file Matrix4f.cpp
/// \brief .cpp file for Matrix4f
///
//
//////////////////////////////////////////////////////////////////////////////

#include "Matrix4f.h"
#include "H3DTypeOperators.h"

using namespace H3D;
using namespace ArithmeticTypes;

/// Constructor. From Rotation object.
Matrix4f::Matrix4f( const Rotation &r ) {
  H3DFloat cosa = H3DCos( r.angle );
  H3DFloat sina = H3DSin( r.angle );
  H3DFloat x = r.axis.x;
  H3DFloat y = r.axis.y;
  H3DFloat z = r.axis.z;
  H3DFloat x2 = x*x;
  H3DFloat y2 = y*y;
  H3DFloat z2 = z*z;
  H3DFloat xy = x*y;
  H3DFloat yz = y*z;
  H3DFloat xz = x*z;
  *this = Matrix4f::Matrix4f( 
    x2 + cosa * (1-x2), xy*(1 - cosa) - z * sina,  xz*(1 - cosa) + y * sina, 0,
    xy*(1 - cosa) + z * sina, y2 + cosa * (1-y2), yz*(1 - cosa) - x * sina,  0,
    xz*(1 - cosa) - y * sina, yz*(1 - cosa) + x * sina, z2 + cosa * (1-z2), 0,
    0, 0, 0, 1 );
}

/// Constructor. From Quaternion object.
Matrix4f::Matrix4f( const Quaternion &q ) {
  H3DFloat x  = (H3DFloat) 2.0*q.v.x;
  H3DFloat y  = (H3DFloat) 2.0*q.v.y;
  H3DFloat z  = (H3DFloat) 2.0*q.v.z;
  H3DFloat xw = x*q.w;
  H3DFloat yw = y*q.w;
  H3DFloat zw = z*q.w;
  H3DFloat xx = x*q.v.x;
  H3DFloat xy = y*q.v.x;
  H3DFloat xz = z*q.v.x;
  H3DFloat yy = y*q.v.y;
  H3DFloat yz = z*q.v.y;
  H3DFloat zz = z*q.v.z;
  
  *this = Matrix4f::Matrix4f( 1 - yy - zz, xy - zw, xz + yw, 0,
                              xy + zw, 1 - xx - zz, yz - xw, 0,
                              xz - yw, yz + xw, 1 - xx - yy, 0,
                              0, 0, 0, 1 );
}

/// Explicit. From Matrix3f object. Creates a transform matrix where
/// the rotation part is the Matrix3f.
Matrix4f::Matrix4f( const Matrix3f &m ) {
  *this = Matrix4f::Matrix4f( m[0][0], m[0][1], m[0][2], 0,
                              m[1][0], m[1][1], m[1][2], 0,
                              m[2][0], m[2][1], m[2][2], 0,
                              0, 0, 0, 1 );
}

Matrix4f::Matrix4f( const Matrix4d &matrix ) {
  m[0][0] = (H3DFloat)matrix[0][0];
  m[0][1] = (H3DFloat)matrix[0][1];
  m[0][2] = (H3DFloat)matrix[0][2];
  m[0][3] = (H3DFloat)matrix[0][3];
  m[1][0] = (H3DFloat)matrix[1][0];
  m[1][1] = (H3DFloat)matrix[1][1];
  m[1][2] = (H3DFloat)matrix[1][2];
  m[1][3] = (H3DFloat)matrix[1][3];
  m[2][0] = (H3DFloat)matrix[2][0];
  m[2][1] = (H3DFloat)matrix[2][1];
  m[2][2] = (H3DFloat)matrix[2][2];
  m[2][3] = (H3DFloat)matrix[2][3];
  m[3][0] = (H3DFloat)matrix[3][0];
  m[3][1] = (H3DFloat)matrix[3][1];
  m[3][2] = (H3DFloat)matrix[3][2];
  m[3][3] = (H3DFloat)matrix[3][3];
}

// This code has been automatically generated from Maple 7, see maple/matrix_inverse.mxs.
Matrix4f Matrix4f::transformInverse() const {
  H3DFloat m00 = m[0][0];
  H3DFloat m01 = m[0][1];
  H3DFloat m02 = m[0][2];
  H3DFloat m03 = m[0][3];
  
  H3DFloat m10 = m[1][0];
  H3DFloat m11 = m[1][1];
  H3DFloat m12 = m[1][2];
  H3DFloat m13 = m[1][3];
  
  H3DFloat m20 = m[2][0];
  H3DFloat m21 = m[2][1];
  H3DFloat m22 = m[2][2];
  H3DFloat m23 = m[2][3];
  
  H3DFloat t4 = m00*m11;
  H3DFloat t6 = m00*m21;
  H3DFloat t8 = m10*m01;
  H3DFloat t10 = m10*m21;
  H3DFloat t12 = m20*m01;
  H3DFloat t14 = m20*m11;
  H3DFloat d = (t4*m22-t6*m12-t8*m22+t10*m02+t12*m12-t14*m02);

  if( H3DAbs(d) == 0 ) {
    throw SingularMatrix4f( "", H3D_FULL_LOCATION );
  }
  H3DFloat t17 = 1/d;

  H3DFloat t20 = m21*m02;
  H3DFloat t23 = m01*m12;
  H3DFloat t24 = m11*m02;
  H3DFloat t43 = m20*m02;
  H3DFloat t46 = m00*m12;
  H3DFloat t47 = m10*m02;
  H3DFloat t51 = m00*m13;
  H3DFloat t54 = m10*m03;
  H3DFloat t57 = m20*m03;
  H3DFloat inv00 = (m11*m22-m21*m12)*t17;
  H3DFloat inv01 = -(m01*m22-t20)*t17;
  H3DFloat inv02 = (t23-t24)*t17;
  H3DFloat inv03 = 
	-(t23*m23-m01*m13*m22-t24*m23+m11*m03*m22+t20*m13-m21*m03* m12)*t17;
  H3DFloat inv10 = -(m10*m22-m20*m12)*t17;
  H3DFloat inv11 = (m00*m22-t43)*t17;
  H3DFloat inv12 = -(t46-t47)*t17;
  H3DFloat inv13 = (t46*m23-t51*m22-t47*m23+t54*m22+t43*m13-t57*m12)*t17;
  H3DFloat inv20 = (t10-t14)*t17;
  H3DFloat inv21 = -(t6-t12)*t17;
  H3DFloat inv22 = (t4-t8)*t17;
  H3DFloat inv23 = -(t4*m23-t51*m21-t8*m23+t54*m21+t12*m13-t57*m11)*t17;
  
  return Matrix4f( inv00, inv01, inv02, inv03,
		   inv10, inv11, inv12, inv13,
		   inv20, inv21, inv22, inv23,
		   0, 0, 0, 1 );  
}

// This code has been automatically generated from Maple 7, see maple/matrix_inverse.mxs.
Matrix4f Matrix4f::inverse() const {
  H3DFloat m30 = m[3][0];
  H3DFloat m31 = m[3][1];
  H3DFloat m32 = m[3][2];
  H3DFloat m33 = m[3][3];

  /// If transform inverse use that version.
  if( H3DAbs( m30 ) < Constants::f_epsilon &&
      H3DAbs( m31 ) < Constants::f_epsilon &&
      H3DAbs( m32 ) < Constants::f_epsilon && 
      H3DAbs( m33 - 1 ) < Constants::f_epsilon ) {
    return( transformInverse() );
  }

  H3DFloat m00 = m[0][0];
  H3DFloat m01 = m[0][1];
  H3DFloat m02 = m[0][2];
  H3DFloat m03 = m[0][3];
  
  H3DFloat m10 = m[1][0];
  H3DFloat m11 = m[1][1];
  H3DFloat m12 = m[1][2];
  H3DFloat m13 = m[1][3];
  
  H3DFloat m20 = m[2][0];
  H3DFloat m21 = m[2][1];
  H3DFloat m22 = m[2][2];
  H3DFloat m23 = m[2][3];

  H3DFloat t14 = m00*m11;
  H3DFloat t15 = m22*m33;
  H3DFloat t17 = m23*m32;
  H3DFloat t19 = m00*m21;
  H3DFloat t20 = m12*m33;
  H3DFloat t22 = m13*m32;
  H3DFloat t24 = m00*m31;
  H3DFloat t25 = m12*m23;
  H3DFloat t27 = m13*m22;
  H3DFloat t29 = m10*m01;
  H3DFloat t32 = m10*m21;
  H3DFloat t33 = m02*m33;
  H3DFloat t35 = m03*m32;
  H3DFloat t37 = m10*m31;
  H3DFloat t38 = m02*m23;
  H3DFloat t40 = m03*m22;
  H3DFloat t42 =	
    t14*t15-t14*t17-t19*t20+t19*t22+t24*t25-t24*t27-
    t29*t15+t29*t17+t32*t33-t32*t35-t37*t38+t37*t40;
  H3DFloat t43 = m20*m01;
  H3DFloat t46 = m20*m11;
  H3DFloat t49 = m20*m31;
  H3DFloat t50 = m02*m13;
  H3DFloat t52 = m03*m12;
  H3DFloat t54 = m30*m01;
  H3DFloat t57 = m30*m11;
  H3DFloat t60 = m30*m21;
  H3DFloat t63 = 
    t43*t20-t43*t22-t46*t33+t46*t35+t49*t50-t49*t52-
    t54*t25+t54*t27+t57*t38-t57*t40-t60*t50+t60*t52;
  
  H3DFloat d = t42+t63;

  if( H3DAbs(d) < Constants::f_epsilon ){
	throw SingularMatrix4f( "", H3D_FULL_LOCATION );
  }

  H3DFloat t65 = 1/d;
  H3DFloat t71 = m21*m02;
  H3DFloat t73 = m03*m21;
  H3DFloat t75 = m02*m31;
  H3DFloat t77 = m03*m31;
  H3DFloat t81 = m01*m12;
  H3DFloat t83 = m01*m13;
  H3DFloat t85 = m11*m02;
  H3DFloat t87 = m03*m11;
  H3DFloat t101 = m10*m22;
  H3DFloat t103 = m10*m23;
  H3DFloat t105 = m20*m12;
  H3DFloat t107 = m20*m13;
  H3DFloat t111 = m30*m13;
  H3DFloat t115 = m00*m22;
  H3DFloat t117 = m00*m23;
  H3DFloat t119 = m20*m02;
  H3DFloat t121 = m20*m03;
  H3DFloat t123 = m30*m02;
  H3DFloat t125 = m30*m03;
  H3DFloat t129 = m00*m12;
  H3DFloat t131 = m00*m13;
  H3DFloat t133 = m10*m02;
  H3DFloat t135 = m10*m03;
  H3DFloat inv00 = 
    (m11*m22*m33-m11*m23*m32-m21*m12*m33+
     m21*m13*m32+m31*m12*m23-m31*m13*m22)*t65;
  H3DFloat inv01 = 
    -(m01*m22*m33-m01*m23*m32-t71*m33+
      t73*m32+t75*m23-t77*m22)*t65;
  H3DFloat inv02 = (t81*m33-t83*m32-t85*m33+t87*m32+t75*m13-t77*m12)*t65;
  H3DFloat inv03 = -(t81*m23-t83*m22-t85*m23+t87*m22+t71*m13-t73*m12)*t65;
  H3DFloat inv10 = 
    -(t101*m33-t103*m32-t105*m33+t107*m32+
      m30*m12*m23-t111*m22)*t65;
  H3DFloat inv11 = 
    (t115*m33-t117*m32-t119*m33+t121*m32+
     t123*m23-t125*m22)*t65;
  H3DFloat inv12 = 
    -(t129*m33-t131*m32-t133*m33+t135*m32+
      t123*m13-t125*m12)*t65;
  H3DFloat inv13 = 
    (t129*m23-t131*m22-t133*m23+t135*m22+
     t119*m13-t121*m12)*t65;
  H3DFloat inv20 = (t32*m33-t103*m31-t46*m33+t107*m31+t57*m23-t111*m21)*t65;
  H3DFloat inv21 = -(t19*m33-t117*m31-t43*m33+t121*m31+t54*m23-t125*m21)*t65;
  H3DFloat inv22 = (t14*m33-t131*m31-t29*m33+t135*m31+t54*m13-t125*m11)*t65;
  H3DFloat inv23 = -(t14*m23-t131*m21-t29*m23+t135*m21+t43*m13-t121*m11)*t65;
  H3DFloat inv30 = -(t32*m32-t101*m31-t46*m32+t105*m31+t57*m22-t60*m12)*t65;
  H3DFloat inv31 = (t19*m32-t115*m31-t43*m32+t119*m31+t54*m22-t60*m02)*t65;
  H3DFloat inv32 = -(t14*m32-t129*m31-t29*m32+t133*m31+t54*m12-t57*m02)*t65;
  H3DFloat inv33 = (t14*m22-t19*m12-t29*m22+t32*m02+t43*m12-t46*m02)*t65;

  return( Matrix4f( inv00, inv01, inv02, inv03,
		    inv10, inv11, inv12, inv13,
		    inv20, inv21, inv22, inv23,
		    inv30, inv31, inv32, inv33 ) );  
}

Matrix3f Matrix4f::getRotationPart() const {
	Matrix3f m = getScaleRotationPart();
	Vec3f x_axis = m * Vec3f(1,0,0);
	Vec3f y_axis = m * Vec3f(0,1,0);
	Vec3f z_axis = m * Vec3f(0,0,1);
	
	x_axis.normalize();
	y_axis.normalize();
	z_axis.normalize();
	
	return Matrix3f( x_axis.x, y_axis.x, z_axis.x,
									 x_axis.y, y_axis.y, z_axis.y,
									 x_axis.z, y_axis.z, z_axis.z );
}

ostream& ArithmeticTypes::operator<<( ostream &os, const Matrix4f &m ) {
  os << m[0][0] << " " << m[0][1] << " " << m[0][2] << " " << m[0][3] << endl;
  os << m[1][0] << " " << m[1][1] << " " << m[1][2] << " " << m[1][3] << endl;
  os << m[2][0] << " " << m[2][1] << " " << m[2][2] << " " << m[2][3] << endl;
  os << m[3][0] << " " << m[3][1] << " " << m[3][2] << " " << m[3][3] << endl;
  return os;
}


