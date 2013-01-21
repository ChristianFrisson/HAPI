//////////////////////////////////////////////////////////////////////////////
//    Copyright 2004-2013, SenseGraphics AB
//
//    This file is part of HAPI.
//
//    HAPI is free software; you can redistribute it and/or modify
//    it under the terms of the GNU General Public License as published by
//    the Free Software Foundation; either version 2 of the License, or
//    (at your option) any later version.
//
//    HAPI is distributed in the hope that it will be useful,
//    but WITHOUT ANY WARRANTY; without even the implied warranty of
//    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//    GNU General Public License for more details.
//
//    You should have received a copy of the GNU General Public License
//    along with HAPI; if not, write to the Free Software
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

#include <HAPI/HapticPointSet.h>
#include <HAPI/PlaneConstraint.h>

#ifdef HAVE_OPENGL
#ifdef MACOSX
#include <OpenGL/gl.h>
#else
#include <GL/gl.h>
#endif
#endif

using namespace HAPI;

bool HapticPointSet::lineIntersectShape( const Vec3 &from, 
                                         const Vec3 &to,
                                         Collision::IntersectionInfo &result,
                                         Collision::FaceType face ) { 
  bool have_intersection = false;
  Collision::IntersectionInfo closest_intersection;
  HAPIFloat min_d2;
  for( unsigned int i = 0; i < points.size(); i++ ) {
    Collision::Point &p = points[i];
    if( p.lineIntersect( from, to, closest_intersection, face ) )	{
      Vec3 v = closest_intersection.point - from;
      HAPIFloat distance_sqr = v * v;
       
      if( !have_intersection ) {
        have_intersection = true;
        result = closest_intersection;
        min_d2 = distance_sqr;
      } else {
        if( distance_sqr < min_d2 ) {
          result = closest_intersection;
          min_d2 = distance_sqr;
        }
      }
    }
  }

  return have_intersection;
}

void HapticPointSet::getConstraintsOfShape( const Vec3 &point,
                                            Constraints &constraints,
                                            Collision::FaceType face,
                                            HAPIFloat radius ) {
  if( points.size() > 0 ) {
    unsigned int size = constraints.size();
    for( unsigned int i = 0; i < points.size(); i++ ) {
      Collision::Point &temp_p = points[i];
      temp_p.getConstraints( point, constraints, face, radius );
    }
    
    for( unsigned int i = size; i < constraints.size(); i ++ ) {
      PlaneConstraint &pc = constraints[i];
      pc.haptic_shape.reset(this);
    }
  }
}

#ifdef HAVE_OPENGL
void HapticPointSet::glRenderShape() {
  glPushAttrib( GL_POINT_BIT | GL_LIGHTING_BIT );
  glDisable( GL_CULL_FACE );
  glDisable( GL_LIGHTING );
  glBegin( GL_POINTS );
  for( unsigned int i = 0; i < points.size(); i++ ) {
    HAPI::Collision::Point &p = points[i];
    glVertex3d( p.position.x, p.position.y, p.position.z );
  }
  glEnd();
  glPopAttrib();
}
#endif

void HapticPointSet::closestPointOnShape( const Vec3 &p,
                                          Vec3 &cp,
                                          Vec3 &n,
                                          Vec3 &tc ) {
  Vec3 temp_cp, temp_n;
  HAPIFloat distance, temp_distance;
  for( unsigned int i = 0; i < points.size(); i++ ) {
    temp_cp = points[i].position;
    if( i == 0 ) {
      cp = temp_cp;
      distance = (cp - p).lengthSqr();
      n = p - temp_cp;
    }
    else {
      temp_n = p - temp_cp;
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
}

bool HapticPointSet::movingSphereIntersectShape( HAPIFloat radius,
                                                const Vec3 &from, 
                                                const Vec3 &to ) {
  for( unsigned int i = 0; i < points.size(); i++ ) {
    if( points[i].movingSphereIntersect( radius, from, to ) ) return true;
  }
  return false;
}

void HapticPointSet::getTangentSpaceMatrixShape( const Vec3 &point,
                                                 Matrix4 &result_mtx ) {
  if( !points.empty() ) {
    if( points.size() == 1 ) {
      points[0].getTangentSpaceMatrix( point, result_mtx );
    } else {
      unsigned int closest_point = 0;
      Vec3 temp_cp, temp_n;
      points[0].closestPoint( point, temp_cp, temp_n, temp_n );
      HAPIFloat distance = ( temp_cp - point).lengthSqr();
      HAPIFloat temp_distance;
      for( unsigned int i = 1; i < points.size(); i++ ) {
        points[i].closestPoint( point, temp_cp, temp_n, temp_n );
        temp_distance = (temp_cp - point).lengthSqr();
        if( temp_distance < distance ) {
          closest_point = i;
          distance = temp_distance;
        }
      }
      points[closest_point].
        getTangentSpaceMatrix( point, result_mtx );
    }   
  }
}
