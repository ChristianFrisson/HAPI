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
/// \file HAPIHapticShape.cpp
/// \brief cpp file for HAPIHapticShape
///
//
//////////////////////////////////////////////////////////////////////////////

#include <HAPI/HAPIHapticShape.h>
#include <HAPI/PlaneConstraint.h>

#ifdef MACOSX
#include <OpenGL/gl.h>
#else
#include <GL/gl.h>
#endif

using namespace HAPI;

int HAPIHapticShape::current_max_id = 0;
list< int > HAPIHapticShape::free_ids;

int HAPIHapticShape::genShapeId() {
  if( free_ids.empty() ) {
    return current_max_id++;
  } else {
    int id = free_ids.front();
    free_ids.pop_front();
    return id;
  }
}

void HAPIHapticShape::delShapeId( int id ) {
  // todo: remove all resources used by id, e.g. hl shape id, done i think.
  free_ids.push_front( id );
}

HAPIHapticShape::~HAPIHapticShape(){
  if( clean_up_func )
    clean_up_func( userdata );
}

void HAPIHapticShape::getConstraints( const Vec3 &point,
                                      Constraints &constraints,
                                      Collision::FaceType face, 
                                      HAPIFloat radius ) {
  if( have_transform ) {
    unsigned int nr_constraints = constraints.size();
    Vec3 scale = inverse.getScalePart();
    HAPIFloat s = max( scale.x, max( scale.y, scale.z ) );    
    getConstraintsOfShape( inverse * point, constraints, face, radius * s );
    for( unsigned int i = nr_constraints; i < constraints.size(); ++i ) {
      PlaneConstraint &pc = constraints[i];
      pc.normal = transform.getRotationPart() * pc.normal;
      pc.point = transform * pc.point;
    }
  } else {
    getConstraintsOfShape( point, constraints, face, radius );
  }
}

void HAPIHapticShape::getConstraintsOfShape( const Vec3 &point,
                                             Constraints &constraints,
                                             Collision::FaceType face, 
                                             HAPIFloat radius ) {
  Vec3 cp, n, tc;
  closestPointOnShape( point, cp, n, tc );
  Vec3 v = cp - point;
  if( radius < 0 || v * v <= radius * radius )
    constraints.push_back( PlaneConstraint( cp, n, tc, this ) );
}


void HAPIHapticShape::closestPoint( const Vec3 &p, Vec3 &cp, 
                                    Vec3 &n, Vec3 &tc ) {
  if( have_transform ) {
    closestPointOnShape( inverse * p, cp, n, tc );
    Matrix4 inv = inverse;
    cp = transform * cp;
    n = transform.getRotationPart() * n;
  } else {
    closestPointOnShape( p, cp, n, tc );
  }
}

bool HAPIHapticShape::lineIntersect( const Vec3 &from, 
                                     const Vec3 &to,
                                     Collision::IntersectionInfo &result,
                                     Collision::FaceType face ) { 
  if( have_transform ) {
    bool have_intersection;
    Vec3 local_from = inverse * from;
    Vec3 local_to = inverse * to;
    have_intersection = 
      lineIntersectShape( local_from, local_to, result, face );
    if( have_intersection ) {
      result.point = transform * result.point;
      result.normal = transform.getRotationPart() * result.normal;
    }
    return have_intersection;
  } else {
    return lineIntersectShape( from, to, result, face );
  }
}

bool HAPIHapticShape::movingSphereIntersect( HAPIFloat radius,
                                             const Vec3 &from, 
                                             const Vec3 &to ) { 
  if( have_transform ) {
    Vec3 scale = inverse.getScalePart();
    Vec3 local_from =  inverse * from;
    Vec3 local_to = inverse * to;
    HAPIFloat s = max( scale.x, max( scale.y, scale.z ) );
    return movingSphereIntersectShape( radius * s,
                                       local_from,
                                       local_to ); 
  } else {
    return movingSphereIntersectShape( radius, from, to ); 
  }
}

void HAPIHapticShape::getTangentSpaceMatrix( const Vec3 &point,
                                             Matrix4 &result_mtx ) {
  if( have_transform ) {
    Vec3 local_point = getInverse() * local_point;
    getTangentSpaceMatrixShape( local_point, result_mtx );
    result_mtx = result_mtx * getInverse();
  } else {
    getTangentSpaceMatrixShape( point, result_mtx );
  }
}

void HAPIHapticShape::glRender() {
  glMatrixMode( GL_MODELVIEW );
  glPushMatrix();
  const Matrix4 &m = transform;
  GLdouble vt[] = { m[0][0], m[1][0], m[2][0], 0,
                    m[0][1], m[1][1], m[2][1], 0,
                    m[0][2], m[1][2], m[2][2], 0,
                    m[0][3], m[1][3], m[2][3], 1 };
  glMultMatrixd( vt );
  glRenderShape();
  glPopMatrix();
}

/// Scale object uniformly on all axis by the given scaling factor.
void HAPIHapticShape::scale( HAPIFloat s ) {
  have_transform = true;
  have_inverse = false;

  transform[0][0] *= s;
  transform[0][1] *= s;
  transform[0][2] *= s;
  transform[0][3] *= s;

  transform[1][0] *= s;
  transform[1][1] *= s;
  transform[1][2] *= s;  
  transform[1][3] *= s;
  
  transform[2][0] *= s;
  transform[2][1] *= s;
  transform[2][2] *= s;
  transform[2][3] *= s;
}

/// Translate object.
void HAPIHapticShape::translate( const Vec3 &t ) {
  have_transform = true;
  have_inverse = false;
  transform[0][3] += t.x;
  transform[1][3] += t.y;
  transform[2][3] += t.z;
}

/// Rotate object.
void HAPIHapticShape::rotate( const Rotation &r ) {
  have_transform = true;
  have_inverse = false;
  transform  = ((Matrix4)r) * transform; 
}
