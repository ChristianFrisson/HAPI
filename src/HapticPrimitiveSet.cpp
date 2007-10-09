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

#include <HAPI/HapticPrimitiveSet.h>
#include <HAPI/PlaneConstraint.h>

using namespace HAPI;

bool HapticPrimitiveSet::lineIntersectShape( 
          const Vec3 &from, 
          const Vec3 &to,
          Collision::IntersectionInfo &result,
          Collision::FaceType face ) { 
  // TODO: find closest?
  bool have_intersection = false;
  Collision::IntersectionInfo closest_intersection;
  HAPIFloat min_d2;
  for( unsigned int i = 0; i < primitives.size(); i++ ) {
    Collision::GeometryPrimitive *a_primitive = primitives[i];
    if( a_primitive->lineIntersect( from, to, result, face ) )	{
      Vec3 v = result.point - from;
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
    result.point = closest_intersection.point;
    result.normal = closest_intersection.normal;
    result.face = closest_intersection.face;
  }
  return have_intersection;
}

void HapticPrimitiveSet::getConstraintsOfShape( const Vec3 &point,
                                         Constraints &constraints,
                                         Collision::FaceType face,
                                         HAPIFloat radius ) {
  if( primitives.size() > 0 ) {
    unsigned int size = constraints.size();
    for( unsigned int i = 0; i < primitives.size(); i++ ) {
      Collision::GeometryPrimitive *a_primitive = primitives[i];
      a_primitive->getConstraints( point, constraints, face, radius );
    }

    for( unsigned int i = size; i < constraints.size(); i ++ ) {
      PlaneConstraint &pc = constraints[i];
      pc.haptic_shape.reset(this);
    }
  }
}

void HapticPrimitiveSet::glRenderShape() {
  // TODO:
}

void HapticPrimitiveSet::closestPointOnShape( const Vec3 &p,
                                              Vec3 &cp,
                                              Vec3 &n,
                                              Vec3 &tc ) {
  Vec3 temp_cp, temp_n, temp_tc;
  HAPIFloat distance, temp_distance;
  for( unsigned int i = 0; i < primitives.size(); i++ ) {
    primitives[i]->closestPoint( p, temp_cp, temp_n, temp_tc );
    if( i == 0 ) {
      cp = temp_cp;
      distance = (cp - p).lengthSqr();
      n = temp_n;
      tc = temp_tc;
    }
    else {
      temp_distance = (temp_cp - p).lengthSqr();
      if( temp_distance < distance ) {
        cp = temp_cp;
        distance = temp_distance;
        n = temp_n;
        tc = temp_tc;
      }
    }
  }
}

bool HapticPrimitiveSet::movingSphereIntersectShape( HAPIFloat radius,
                                                     const Vec3 &from, 
                                                     const Vec3 &to ) {
  for( unsigned int i = 0; i < primitives.size(); i++ ) {
    if( primitives[i]->movingSphereIntersect( radius, from, to ) ) return true;
  }
  return false;
}
