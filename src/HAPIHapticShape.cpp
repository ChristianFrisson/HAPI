//////////////////////////////////////////////////////////////////////////////
//    Copyright 2004-2019, SenseGraphics AB
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
/// \file HAPIHapticShape.cpp
/// \brief cpp file for HAPIHapticShape
///
//
//////////////////////////////////////////////////////////////////////////////

#include <HAPI/HAPIHapticShape.h>
#include <HAPI/PlaneConstraint.h>
#include <H3DUtil/Rotationd.h>
#include <H3DUtil/H3DMath.h>


#ifdef HAVE_OPENGL
#ifdef MACOSX
#include <OpenGL/gl.h>
#else
#include <GL/gl.h>
#endif
#endif

using namespace HAPI;

int HAPIHapticShape::current_max_id = 0;
std::list< int > HAPIHapticShape::free_ids;

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
    Vec3 local_point = toLocal( point );
    Vec3 scale = inverse.getScalePart();
    HAPIFloat s = H3DMax( scale.x, H3DMax( scale.y, scale.z ) );    
    getConstraintsOfShape( local_point, constraints, face, radius * s );
    if( non_uniform_scaling ) {
      for( unsigned int i = nr_constraints; i < constraints.size(); ++i ) {
        PlaneConstraint &pc = constraints[i];
        pc.normal = transform.getRotationPart() * pc.normal;
        pc.normal.x *= scale.x;
        pc.normal.y *= scale.y;
        pc.normal.z *= scale.z;
        pc.normal = inverse.getRotationPart() * pc.normal;
        pc.normal = transform.getRotationPart() * pc.normal;
        pc.normal.normalizeSafe();
        pc.point = transform * pc.point;
      }
    } else {
      for( unsigned int i = nr_constraints; i < constraints.size(); ++i ) {
        PlaneConstraint &pc = constraints[i];
        pc.normal = transform.getRotationPart() * pc.normal;
        pc.point = transform * pc.point;
      }
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
    cp = transform * cp;
    if( non_uniform_scaling ) {
      Vec3 scaling = inverse.getScalePart();
      n = transform.getRotationPart() * n;
      n.x *= scaling.x;
      n.y *= scaling.y;
      n.z *= scaling.z;
      n = inverse.getRotationPart() * n;
      n = transform.getRotationPart() * n;
      n.normalizeSafe();
    } else {
      n = transform.getRotationPart() * n;
    }
  } else {
    closestPointOnShape( p, cp, n, tc );
  }
}

void HAPIHapticShape::transformShape( const Matrix4 &t ) {
  setTransform( t * transform );
}

void HAPIHapticShape::setTransform( const Matrix4 &t ) {
  have_transform = true;
  have_inverse = true;
  transform = t;

  position = t.getTranslationPart();
  rotation = Rotation( t.getRotationPart() );
  scale_factor = t.getScalePart();
  inverse = transform.inverse();
  setNonUniformScalingFlag();
}

bool HAPIHapticShape::isDynamic() {
  return ( forced_dynamic ||
           velocity != Vec3( 0, 0, 0 ) ||
           angular_velocity.angle != 0 ||
           growth_rate != Vec3( 0, 0, 0 ) );
    
}

const Matrix4 &HAPIHapticShape::getInverse() {
  if( !have_inverse ) {
    inverse = transform.inverse();
    have_inverse = true;
  }
  return inverse;
}

Matrix4 HAPIHapticShape::getTransform() {
  return transform;
}

void HAPIHapticShape::initializeTransfer( HAPIHapticShape *s ) {
  if( s ) {
    last_transform = s->last_transform; //getTransform();
    last_inverse = s->last_inverse; //getInverse();
  } else {
    last_transform = getTransform();
    last_inverse = getInverse();
  }
}

void HAPIHapticShape::moveTimestep( HAPITime dt ) {
  if( !isDynamic() ) {
    // Avoid small floating point errors that might
    // lead to fall-through for static shapes
    return;
  }

  // save the previous transforms.
  last_transform = getTransform();
  last_inverse = getInverse();

  have_transform = true;
  have_inverse = false;
  
  position += velocity*dt;
  rotation = (angular_velocity * dt) * rotation; 
  scale_factor += growth_rate * dt;

  updateTransform();
}


Vec3 HAPIHapticShape::moveTimestep( const HAPI::Vec3 &p, HAPITime dt ) {
  // move to local space
  Vec3 new_p = getInverse() * p;

  // calculate matrix for next time step.
  Vec3 new_pos = position +  velocity*dt;
  Rotation new_rot = (angular_velocity * dt) * rotation; 
  Vec3 new_scale = scale_factor + growth_rate * dt;

  Matrix4  new_transform = Matrix4( new_scale.x, 0, 0, 0,
                                    0, new_scale.y, 0, 0,
                                    0, 0, new_scale.z, 0,
                                    0, 0, 0, 1 );
  new_transform = ((Matrix4)new_rot) * new_transform;
  new_transform[0][3] += new_pos.x;
  new_transform[1][3] += new_pos.y;
  new_transform[2][3] += new_pos.z;
  
  // move back to global space.
  return new_transform * new_p; 
}

Vec3 HAPIHapticShape::moveTimestepVec( const HAPI::Vec3 &p, HAPITime dt ) {
  return (angular_velocity * dt) * p;
}

Vec3 HAPIHapticShape::toGlobal( const Vec3 &p ) {

  Vec3 new_p = p;

  new_p = getTransform() * new_p;

  /*
  // scale
  new_p = new_p * shape->getScale();

  // rotate
  new_p = shape->getRotation() * new_p;

  // translate to position
  new_p = new_p + shape->getPosition();

  */
  return new_p;
}

Vec3 HAPIHapticShape::toLocal( const Vec3 &p ) {
  Vec3 new_p = p;
  
  new_p = getInverse() * new_p;

  /*
  // translate
  new_p = new_p - shape->getPosition();

  // rotate
  new_p = (-shape->getRotation()) * new_p;

  // scale
  new_p = new_p / shape->getScale();
  */

  return new_p;
}

bool HAPIHapticShape::lineIntersect( const Vec3 &from, 
                                     const Vec3 &to,
                                     Collision::IntersectionInfo &result,
                                     Collision::FaceType face,
                                     bool consider_movement ) { 
  if( have_transform ) {
    bool have_intersection;
    Vec3 local_from =  
      consider_movement ? last_inverse * from : toLocal( from );
    Vec3 local_to = toLocal( to );

    have_intersection = 
      lineIntersectShape( local_from, local_to, result, face );

    //cerr << local_from.x << " " << local_to.x << " " << have_intersection << endl;
    if( have_intersection ) {
      result.point = toGlobal( result.point );
      if( non_uniform_scaling ) {
        Vec3 scale = inverse.getScalePart();
        result.normal = rotation * result.normal;
        result.normal.x *= scale.x;
        result.normal.y *= scale.y;
        result.normal.z *= scale.z;
        result.normal = inverse.getRotationPart() * result.normal;
        result.normal = transform.getRotationPart() * result.normal;
        result.normal.normalizeSafe();
      } else {
        result.normal = toGlobalVec( result.normal );
      }
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
    HAPIFloat s = H3DMax( scale.x, H3DMax( scale.y, scale.z ) );
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
    Vec3 local_point = getInverse() * point;
    getTangentSpaceMatrixShape( local_point, result_mtx );
    result_mtx = result_mtx * getInverse();
  } else {
    getTangentSpaceMatrixShape( point, result_mtx );
  }
}

#ifdef HAVE_OPENGL
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
  glMatrixMode( GL_MODELVIEW );
  glPopMatrix();
}
#endif

/// Scale object uniformly on all axis by the given scaling factor.
void HAPIHapticShape::scale( HAPIFloat s ) {
  have_transform = true;
  have_inverse = false;

  scale_factor = scale_factor * s;
  updateTransform();
  setNonUniformScalingFlag();
}

/// Translate object.
void HAPIHapticShape::translate( const Vec3 &t ) {
  have_transform = true;
  have_inverse = false;
  position += t;
  transform[0][3] += t.x;
  transform[1][3] += t.y;
  transform[2][3] += t.z;
}

/// Rotate object.
void HAPIHapticShape::rotate( const Rotation &r ) {
  have_transform = true;
  have_inverse = false;
  rotation = r * rotation;
  updateTransform();
}

void HAPIHapticShape::updateTransform() {
  transform = Matrix4( scale_factor.x, 0, 0, 0,
                       0, scale_factor.y, 0, 0,
                       0, 0, scale_factor.z, 0,
                       0, 0, 0, 1 );
  transform = ((Matrix4)rotation) * transform;
  transform[0][3] += position.x;
  transform[1][3] += position.y;
  transform[2][3] += position.z;
  inverse = transform.inverse();
  have_inverse = true;
}
