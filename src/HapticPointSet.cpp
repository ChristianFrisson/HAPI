//////////////////////////////////////////////////////////////////////////////
//    Copyright 2004-2007, SenseGraphics AB
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
/// \file HapticPointSet.cpp
/// \brief cpp file for HapticPointSet
///
//
//////////////////////////////////////////////////////////////////////////////

#include <HapticPointSet.h>
#include <PlaneConstraint.h>

using namespace HAPI;

bool HapticPointSet::lineIntersect( const Vec3 &from, 
                                       const Vec3 &to,
                                       Bounds::IntersectionInfo &result,
                                       Bounds::FaceType face ) { 
  Matrix4 inv = transform.inverse();
  // TODO: find closest?
  bool have_intersection = false;
  Bounds::IntersectionInfo closest_intersection;
  HAPIFloat min_d2;
  Vec3 from_local = inv * from;
  Vec3 to_local = inv * to;
  for( unsigned int i = 0; i < points.size(); i++ ) {
    Bounds::Point &p = points[i];
    if( p.lineIntersect( from_local, to_local, result, face ) )	{
      Vec3 v = result.point - from_local;
      HAPIFloat distance_sqr = v * v;
       
      if( !have_intersection ) {
        have_intersection = true;
        closest_intersection = result;
        min_d2 = distance_sqr;
      } else {
        if( distance_sqr < min_d2 ) {
          closest_intersection = result;
          min_d2 = distance_sqr;
        }
      }
    }
  }
  
  if( have_intersection ) {
    result.point = transform * closest_intersection.point;
    result.normal = transform.getRotationPart() * closest_intersection.normal;
    result.face = closest_intersection.face;
  }
  return have_intersection;
}

void HapticPointSet::getConstraints( const Vec3 &point,
                                        std::vector< PlaneConstraint > &constraints,
                                        Bounds::FaceType face ) {
  if( points.size() > 0 ) {
    // TODO: check if transform has uniform scale
    bool uniform_scale = true;

    if( uniform_scale ) {
      Vec3 p = transform.inverse() * point;
      unsigned int size = constraints.size();
      for( unsigned int i = 0; i < points.size(); i++ ) {
        Bounds::Point &temp_p = points[i];
        temp_p.getConstraints( p, constraints, face );
      }

      for( unsigned int i = size; i < constraints.size(); i ++ ) {
        PlaneConstraint &pc = constraints[i];
        pc.normal = transform.getScaleRotationPart() * pc.normal;
        pc.normal.normalizeSafe();
        pc.point = transform * pc.point;
        pc.haptic_shape.reset(this);
      }
    } else {
      // TODO: fix this
      unsigned int size = constraints.size();
      for( unsigned int i = 0; i < points.size(); i++ ) {
        Bounds::Point &p = points[i];
        p.getConstraints( point, constraints, face );
      }
      for( unsigned int i = size; i < constraints.size(); i ++ ) {
        PlaneConstraint &pc = constraints[i];
        pc.point = pc.point;
        pc.haptic_shape.reset(this);
      }
    }
  }
}

void HapticPointSet::glRender() {
  glMatrixMode( GL_MODELVIEW );
  glPushMatrix();
  glPushAttrib( GL_POINT_BIT | GL_LIGHTING_BIT );
  glDisable( GL_CULL_FACE );
  glDisable( GL_LIGHTING );
  const Matrix4 &m = transform;
  GLdouble vt[] = { m[0][0], m[1][0], m[2][0], 0,
                    m[0][1], m[1][1], m[2][1], 0,
                    m[0][2], m[1][2], m[2][2], 0,
                    m[0][3], m[1][3], m[2][3], 1 };
  glMultMatrixd( vt );
  glBegin( GL_POINTS );
  for( unsigned int i = 0; i < points.size(); i++ ) {
    HAPI::Bounds::Point &p = points[i];
    glVertex3d( p.position.x, p.position.y, p.position.z );
  }
  glEnd();
  glPopAttrib();
  glPopMatrix();
}

void HapticPointSet::closestPoint( const Vec3 &p,
                                      Vec3 &cp,
                                      Vec3 &n,
                                      Vec3 &tc ) {
  Vec3 temp_cp, temp_n;
  Vec3 local_pos = transform.inverse() * p;
  HAPIFloat distance, temp_distance;
  for( unsigned int i = 0; i < points.size(); i++ ) {
    temp_cp = points[i].position;
    if( i == 0 ) {
      cp = temp_cp;
      distance = (cp - local_pos).lengthSqr();
      n = local_pos - temp_cp;
    }
    else {
      temp_n = local_pos - temp_cp;
      temp_distance = temp_n.lengthSqr();
      if( temp_distance < distance ) {
        cp = temp_cp;
        distance = temp_distance;
        if( temp_distance > Constants::epsilon ) {
          temp_n = temp_n / H3DUtil::H3DSqrt( temp_distance );
        }
        n = temp_n;
      }
    }
  }
  n = transform.getRotationPart() * n;
  cp = transform * cp;
}
