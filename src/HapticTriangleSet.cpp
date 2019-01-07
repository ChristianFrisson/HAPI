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
/// \file HapticTriangleSet.cpp
/// \brief cpp file for HapticTriangleSet
///
//
//////////////////////////////////////////////////////////////////////////////

#include <HAPI/HapticTriangleSet.h>
#include <HAPI/PlaneConstraint.h>

using namespace HAPI;

bool HapticTriangleSet::lineIntersectShape( const Vec3 &from, 
                                            const Vec3 &to,
                                            Collision::IntersectionInfo &result,
                                            Collision::FaceType face ) { 
  bool have_intersection = false;
  Collision::IntersectionInfo closest_intersection;
  HAPIFloat min_d2;

  for( std::vector< Collision::Triangle >::iterator i = triangles.begin();
       i != triangles.end(); ++i ) {
    Collision::Triangle &t = (*i);
    if( t.lineIntersect( from, to, closest_intersection, face ) ) {
      Vec3 v = closest_intersection.point - from;
      HAPIFloat distance_sqr = v * v;
       
      if( !have_intersection ) {
        have_intersection = true;
        result = closest_intersection;
        // if we have a convex shape we only have one intersection
        if( ( convex == CONVEX_FRONT && 
              result.face == Collision::FRONT ) ||
            ( convex == CONVEX_BACK && 
              result.face == Collision::BACK ) ) {
          break;
        }
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

void HapticTriangleSet::getConstraintsOfShape( const Vec3 &point,
                                               Constraints &constraints,
                                               Collision::FaceType face,
                                               HAPIFloat radius ) {
  if( triangles.size() > 0 ) {
    unsigned int size = constraints.size();
    
    if( ( convex == CONVEX_FRONT && 
          face == Collision::FRONT ) ||
        ( convex == CONVEX_BACK && 
          face == Collision::BACK ) ) {
      PlaneConstraint constraint;
      PlaneConstraint best;
      HAPIFloat best_sqr_dist = 0;
      bool first_constraint = true;
      
      // triangle set is convex so only need the closest constraint
      for( std::vector< Collision::Triangle >::iterator i = triangles.begin();
           i != triangles.end(); ++i ) {
        //unsigned int i = 0; i < triangles.size(); ++i ) {
        Collision::Triangle &t = (*i); //*triangles[i];
        
        if( t.getConstraint( point, &constraint, face ) ) {
          if( first_constraint ) {
            best = constraint;
            Vec3 v = point - best.point;
            best_sqr_dist = v * v; 
            first_constraint = false;
          } else {
            Vec3 v = point - constraint.point;
            HAPIFloat sqr_dist = v * v; 
            if( sqr_dist < best_sqr_dist ) {
              best = constraint;
              best_sqr_dist = sqr_dist;
            }
          }
        }
        //constraint.clear();
      }
      if( !first_constraint ) constraints.push_back( best );
    } else {
      // not convex, so add constraints from all triangles.
      for( unsigned int i = 0; i < triangles.size(); ++i ) {
        Collision::Triangle &t = triangles[i];
        t.getConstraints( point, constraints, face, radius  );
      }
    }
    
    for( unsigned int i = size; i < constraints.size(); ++i ) {
      PlaneConstraint &pc = constraints[i];
      pc.haptic_shape.reset(this);
    }
  }
}

#ifdef HAVE_OPENGL
void HapticTriangleSet::glRenderShape() {
  for( unsigned int i = 0; i < triangles.size(); ++i ) {
    triangles[i].render();
  }
}
#endif

void HapticTriangleSet::closestPointOnShape( const Vec3 &p,
                                             Vec3 &cp,
                                             Vec3 &n,
                                             Vec3 &tc ) {
  Vec3 temp_cp, temp_n, temp_tc;
  HAPIFloat distance, temp_distance;
  for( unsigned int i = 0; i < triangles.size(); ++i ) {
    triangles[i].closestPoint( p, temp_cp, temp_n, temp_tc );
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

bool HapticTriangleSet::movingSphereIntersectShape( HAPIFloat radius,
                                                    const Vec3 &from, 
                                                    const Vec3 &to ) {
  for( unsigned int i = 0; i < triangles.size(); ++i ) {
    if( triangles[i].movingSphereIntersect( radius, from, to ) ) return true;
  }
  return false;
}

void HapticTriangleSet::getTangentSpaceMatrixShape( const Vec3 &point,
                                                    Matrix4 &result_mtx ) {
  if( !triangles.empty() ) {
    if( triangles.size() == 1 ) {
      triangles[0].getTangentSpaceMatrix( point, result_mtx );
    } else {
      unsigned int closest_triangle = 0;
      Vec3 temp_cp, temp_n;
      triangles[0].closestPoint( point, temp_cp, temp_n, temp_n );
      HAPIFloat distance = ( temp_cp - point).lengthSqr();
      HAPIFloat temp_distance;
      for( unsigned int i = 1; i < triangles.size(); ++i ) {
        triangles[i].closestPoint( point, temp_cp, temp_n, temp_n );
        temp_distance = (temp_cp - point).lengthSqr();
        if( temp_distance < distance ) {
          closest_triangle = i;
          distance = temp_distance;
        }
      }
      triangles[closest_triangle].
        getTangentSpaceMatrix( point, result_mtx );
    }
  }
}
