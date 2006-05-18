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
/// \file Matrix3d.cpp
/// \brief .cpp file for Matrix3d.
///
//
//////////////////////////////////////////////////////////////////////////////

#include "Matrix3d.h"
#include "Matrix3f.h"
#include "Rotation.h"
#include "Quaternion.h"

using namespace H3D;
using namespace ArithmeticTypes;

Matrix3d Matrix3d::inverse() const {
  
  H3DDouble m00 = m[0][0];
  H3DDouble m01 = m[0][1];
  H3DDouble m02 = m[0][2];
  
  H3DDouble m10 = m[1][0];
  H3DDouble m11 = m[1][1];
  H3DDouble m12 = m[1][2];
  
  H3DDouble m20 = m[2][0];
  H3DDouble m21 = m[2][1];
  H3DDouble m22 = m[2][2];
  H3DDouble t4 = m00*m11;
  H3DDouble t6 = m00*m21;
  H3DDouble t8 = m10*m01;
  H3DDouble t10 = m10*m21;
  H3DDouble t12 = m20*m01;
  H3DDouble t14 = m20*m11;
  H3DDouble d = (t4*m22-t6*m12-t8*m22+t10*m02+t12*m12-t14*m02);
  if( H3DAbs(d) == 0 ) 
	throw SingularMatrix3d( "", H3D_FULL_LOCATION );
  H3DDouble t17 = 1/d;
  H3DDouble inv00 = (m11*m22-m21*m12)*t17;
  H3DDouble inv01 = -(m01*m22-m21*m02)*t17;
  H3DDouble inv02 = (m01*m12-m11*m02)*t17;
  H3DDouble inv10 = -(m10*m22-m20*m12)*t17;
  H3DDouble inv11 = (m00*m22-m20*m02)*t17;
  H3DDouble inv12 = -(m00*m12-m10*m02)*t17;
  H3DDouble inv20 = (t10-t14)*t17;
  H3DDouble inv21 = -(t6-t12)*t17;
  H3DDouble inv22 = (t4-t8)*t17;

  return Matrix3d( inv00, inv01, inv02, 
		   inv10, inv11, inv12, 
		   inv20, inv21, inv22 );
}


/// Constructor. From Rotation object.
Matrix3d::Matrix3d( const Rotation &r ) {
  H3DDouble cosa = H3DCos( r.angle );
  H3DDouble sina = H3DSin( r.angle );
  H3DDouble x = r.axis.x;
  H3DDouble y = r.axis.y;
  H3DDouble z = r.axis.z;
  H3DDouble x2 = x*x;
  H3DDouble y2 = y*y;
  H3DDouble z2 = z*z;
  H3DDouble xy = x*y;
  H3DDouble yz = y*z;
  H3DDouble xz = x*z;
  
  m[0][0] = x2 + cosa * (1-x2);
  m[0][1] = xy*(1 - cosa) - z * sina;
  m[0][2] = xz*(1 - cosa) + y * sina;
  m[1][0] = xy*(1 - cosa) + z * sina;
  m[1][1] = y2 + cosa * (1-y2);
  m[1][2] = yz*(1 - cosa) - x * sina;
  m[2][0] = xz*(1 - cosa) - y * sina;
  m[2][1] = yz*(1 - cosa) + x * sina;
  m[2][2] = z2 + cosa * (1-z2);
}

/// Constructor. From Quaternion object.
Matrix3d::Matrix3d( const Quaternion &q ) {
  H3DDouble x  = (H3DDouble) 2.0*q.v.x;
  H3DDouble y  = (H3DDouble) 2.0*q.v.y;
  H3DDouble z  = (H3DDouble) 2.0*q.v.z;
  H3DDouble xw = x*q.w;
  H3DDouble yw = y*q.w;
  H3DDouble zw = z*q.w;
  H3DDouble xx = x*q.v.x;
  H3DDouble xy = y*q.v.x;
  H3DDouble xz = z*q.v.x;
  H3DDouble yy = y*q.v.y;
  H3DDouble yz = z*q.v.y;
  H3DDouble zz = z*q.v.z;
  
  *this = Matrix3d::Matrix3d( 1 - yy - zz, xy - zw, xz + yw,
                              xy + zw, 1 - xx - zz, yz - xw,
                              xz - yw, yz + xw, 1 - xx - yy );
}

Matrix3d::Matrix3d( const Matrix3f &matrix ) {
  m[0][0] = matrix[0][0];
  m[0][1] = matrix[0][1];
  m[0][2] = matrix[0][2];
  m[1][0] = matrix[1][0];
  m[1][1] = matrix[1][1];
  m[1][2] = matrix[1][2];
  m[2][0] = matrix[2][0];
  m[2][1] = matrix[2][1];
  m[2][2] = matrix[2][2];
}

Vec3d Matrix3d::toEulerAngles() {
  Vec3d v;
  H3DDouble sint = -m[2][0];
  if( sint < -1 ) sint = -1;
  if( sint > 1 ) sint = 1;
  H3DDouble cost = H3DSqrt( 1- sint*sint );
  H3DDouble sinv, cosv, sinf, cosf;
 
  if ( H3DAbs( cost ) < 1E-9 ) {
    sinv = - m[1][2];
    cosv = m[1][1];
    sinf = 0.0;
    cosf = 1.0;
  }
  else {
    sinv = m[2][1]/cost;
    cosv = m[2][2]/cost;
    sinf = m[1][0]/cost;
    cosf = m[0][0]/cost;
  }
  v.x = H3DAtan2( sinv,cosv );
  v.y = H3DAtan2( sint,cost );
  v.z = H3DAtan2( sinf,cosf );
  return v;
}

ostream& H3D::ArithmeticTypes::operator<<( ostream &os, const Matrix3d &m ) {
  os << m[0][0] << " " << m[0][1] << " " << m[0][2] << endl;
  os << m[1][0] << " " << m[1][1] << " " << m[1][2] << endl;
  os << m[2][0] << " " << m[2][1] << " " << m[2][2] << endl;
  return os;
}


