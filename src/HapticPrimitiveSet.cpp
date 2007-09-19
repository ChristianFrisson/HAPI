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
/// \file HapticPrimitiveSet.cpp
/// \brief cpp file for HapticPrimitiveSet
///
//
//////////////////////////////////////////////////////////////////////////////

#include <HapticPrimitiveSet.h>
#include <PlaneConstraint.h>

using namespace HAPI;

bool HapticPrimitiveSet::lineIntersect( const Vec3 &from, 
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
  for( unsigned int i = 0; i < primitives.size(); i++ ) {
    Bounds::GeometryPrimitive *a_primitive = primitives[i];
    if( a_primitive->lineIntersect( from_local, to_local, result, face ) )	{
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

void HapticPrimitiveSet::getConstraints( const Vec3 &point,
                                         Constraints &constraints,
                                         Bounds::FaceType face,
                                         HAPIFloat radius ) {
  if( primitives.size() > 0 ) {
    // TODO: check if transform has uniform scale
    bool uniform_scale = true;

    if( uniform_scale ) {
      Matrix4 inverse =  transform.inverse();
      Vec3 p = inverse * point;

      Vec3 s = inverse.getScalePart();
        // uniform scaling so use any component
      HAPIFloat r = radius * s.x;

      unsigned int size = constraints.size();
      for( unsigned int i = 0; i < primitives.size(); i++ ) {
        Bounds::GeometryPrimitive *a_primitive = primitives[i];
        a_primitive->getConstraints( p, constraints, face, r );
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
      for( unsigned int i = 0; i < primitives.size(); i++ ) {
        Bounds::GeometryPrimitive *a_primitive = primitives[i];
        a_primitive->getConstraints( point, constraints, face /*r*/);
      }
      for( unsigned int i = size; i < constraints.size(); i ++ ) {
        PlaneConstraint &pc = constraints[i];
        pc.point = pc.point;
        pc.haptic_shape.reset(this);
      }
    }
  }
}

void HapticPrimitiveSet::glRender() {
}

void HapticPrimitiveSet::closestPoint( const Vec3 &p,
                                      Vec3 &cp,
                                      Vec3 &n,
                                      Vec3 &tc ) {
  Vec3 temp_cp, temp_n, temp_tc;
  Vec3 local_pos = transform.inverse() * p;
  HAPIFloat distance, temp_distance;
  for( unsigned int i = 0; i < primitives.size(); i++ ) {
    primitives[i]->closestPoint( local_pos, temp_cp, temp_n, temp_tc );
    if( i == 0 ) {
      cp = temp_cp;
      distance = (cp - local_pos).lengthSqr();
      n = temp_n;
      tc = temp_tc;
    }
    else {
      temp_distance = (temp_cp - local_pos).lengthSqr();
      if( temp_distance < distance ) {
        cp = temp_cp;
        distance = temp_distance;
        n = temp_n;
        tc = temp_tc;
      }
    }
  }
  n = transform.getRotationPart() * n;
  cp = transform * cp;
}
