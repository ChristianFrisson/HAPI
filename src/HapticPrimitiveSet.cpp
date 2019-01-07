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
  bool have_intersection = false;
  Collision::IntersectionInfo closest_intersection;
  HAPIFloat min_d2;
  for( unsigned int i = 0; i < primitives.size(); ++i ) {
    Collision::GeometryPrimitive *a_primitive = primitives[i];
    if( a_primitive->lineIntersect( from, to, closest_intersection, face ) ) {
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

void HapticPrimitiveSet::getConstraintsOfShape( const Vec3 &point,
                                         Constraints &constraints,
                                         Collision::FaceType face,
                                         HAPIFloat radius ) {
  if( primitives.size() > 0 ) {
    unsigned int size = constraints.size();
    for( unsigned int i = 0; i < primitives.size(); ++i ) {
      Collision::GeometryPrimitive *a_primitive = primitives[i];
      a_primitive->getConstraints( point, constraints, face, radius );
    }

    for( unsigned int i = size; i < constraints.size(); ++i ) {
      PlaneConstraint &pc = constraints[i];
      pc.haptic_shape.reset(this);
    }
  }
}

#ifdef HAVE_OPENGL
void HapticPrimitiveSet::glRenderShape() {
  for( PrimitiveVector::const_iterator i = primitives.begin();
       i != primitives.end(); ++i ) {
    (*i)->render();
  }
}
#endif

void HapticPrimitiveSet::closestPointOnShape( const Vec3 &p,
                                              Vec3 &cp,
                                              Vec3 &n,
                                              Vec3 &tc ) {
  Vec3 temp_cp, temp_n, temp_tc;
  HAPIFloat distance, temp_distance;
  for( unsigned int i = 0; i < primitives.size(); ++i ) {
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
  for( unsigned int i = 0; i < primitives.size(); ++i ) {
    if( primitives[i]->movingSphereIntersect( radius, from, to ) ) return true;
  }
  return false;
}

void HapticPrimitiveSet::getTangentSpaceMatrixShape( const Vec3 &point,
                                                     Matrix4 &result_mtx ) {
  if( !primitives.empty() ) {
    if( primitives.size() == 1 ) {
      primitives[0]->getTangentSpaceMatrix( point, result_mtx );
    } else {
      unsigned int closest_primitive = 0;
      Vec3 temp_cp, temp_n;
      primitives[0]->closestPoint( point, temp_cp, temp_n, temp_n );
      HAPIFloat distance = ( temp_cp - point).lengthSqr();
      HAPIFloat temp_distance;
      for( unsigned int i = 1; i < primitives.size(); ++i ) {
        primitives[i]->closestPoint( point, temp_cp, temp_n, temp_n );
        temp_distance = (temp_cp - point).lengthSqr();
        if( temp_distance < distance ) {
          closest_primitive = i;
          distance = temp_distance;
        }
      }
      primitives[closest_primitive]->
        getTangentSpaceMatrix( point, result_mtx );
    }
  }
}

#ifdef HAVE_OPENGL
void HapticPrimitiveSet::countNrOfPrimitives() {
  // If I did not miss anything we can do this because the
  // primitives vector will not change so that a primitive
  // changes to a primitive of another type or the size
  // of the vector is changed.
  nr_triangles = 0;
  nr_points = 0;
  nr_lines = 0;
  if( !primitives.empty() ) {
    for( PrimitiveVector::const_iterator i = primitives.begin();
         i != primitives.end(); ++i ) {
      if( dynamic_cast< Collision::LineSegment * >( *i ) )
        ++nr_lines;
      else if( dynamic_cast< Collision::Triangle * >( *i ) )
        ++nr_triangles;
      else if( dynamic_cast< Collision::Point * >( *i ) )
        ++nr_points;
    }
    if( nr_triangles == 0 && nr_points == 0 && nr_lines == 0 ) {
      // to indicate that we have no idea how many of each there is.
      nr_triangles = -1;
      nr_points = -1;
      nr_lines = -1;
    }
  }
}
#endif
