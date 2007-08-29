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
/// \file HapticTriangleSet.cpp
/// \brief cpp file for HapticTriangleSet
///
//
//////////////////////////////////////////////////////////////////////////////

#include <HapticTriangleSet.h>
#include <PlaneConstraint.h>

using namespace HAPI;

bool HapticTriangleSet::lineIntersect( const Vec3 &from, 
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
   for( vector< Bounds::Triangle >::iterator i = triangles.begin();
             i != triangles.end(); ++i ) {
          //unsigned int i = 0; i < triangles.size(); i++ ) {
          Bounds::Triangle &t = (*i); //*triangles[i];
    if( t.lineIntersect( from_local, to_local, result, face ) )	{
      Vec3 v = result.point - from_local;
      HAPIFloat distance_sqr = v * v;
       
      if( !have_intersection ) {
        have_intersection = true;
        closest_intersection = result;
        // if we have a convex shape we only have one intersection
        if( ( convex == CONVEX_FRONT && 
              result.face == Bounds::FRONT ) ||
            ( convex == CONVEX_BACK && 
              result.face == Bounds::BACK ) ) {
          break;
        }
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

void HapticTriangleSet::getConstraints( const Vec3 &point,
                                        Constraints &constraints,
                                        Bounds::FaceType face ) {
  if( triangles.size() > 0 ) {
    // TODO: check if transform has uniform scale
    bool uniform_scale = true;

    if( uniform_scale ) {
      Vec3 p = transform.inverse() * point;
      unsigned int size = constraints.size();

      if( ( convex == CONVEX_FRONT && 
            face == Bounds::FRONT ) ||
          ( convex == CONVEX_BACK && 
            face == Bounds::BACK ) ) {
        PlaneConstraint constraint;
        PlaneConstraint best;
        HAPIFloat best_sqr_dist = 0;
        bool first_constraint = true;

        for( vector< Bounds::Triangle >::iterator i = triangles.begin();
             i != triangles.end(); ++i ) {
          //unsigned int i = 0; i < triangles.size(); i++ ) {
          Bounds::Triangle &t = (*i); //*triangles[i];
          
          if( t.getConstraint( p, &constraint, face ) ) {
            if( first_constraint ) {
              best = constraint;
              Vec3 v = p - best.point;
              best_sqr_dist = v * v; 
              first_constraint = false;
            } else {
              Vec3 v = p - constraint.point;
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
        for( unsigned int i = 0; i < triangles.size(); i++ ) {
          Bounds::Triangle &t = triangles[i];
          t.getConstraints( p, constraints, face );
        }
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
      for( unsigned int i = 0; i < triangles.size(); i++ ) {
        Bounds::Triangle &t = triangles[i];
        t.getConstraints( point, constraints, face );
      }
      for( unsigned int i = size; i < constraints.size(); i ++ ) {
        PlaneConstraint &pc = constraints[i];
        pc.point = pc.point;
        pc.haptic_shape.reset(this);
      }
    }
  }
}

void HapticTriangleSet::glRender() {
  glMatrixMode( GL_MODELVIEW );
  glPushMatrix();
  glPushAttrib( GL_POLYGON_BIT );
  glDisable( GL_CULL_FACE );
  const Matrix4 &m = transform;
  GLdouble vt[] = { m[0][0], m[1][0], m[2][0], 0,
                    m[0][1], m[1][1], m[2][1], 0,
                    m[0][2], m[1][2], m[2][2], 0,
                    m[0][3], m[1][3], m[2][3], 1 };
  glMultMatrixd( vt );
  glBegin( GL_TRIANGLES );
  for( unsigned int i = 0; i < triangles.size(); i++ ) {
    HAPI::Bounds::Triangle &t = triangles[i];
    glVertex3d( t.a.x, t.a.y, t.a.z );
    glVertex3d( t.b.x, t.b.y, t.b.z );
    glVertex3d( t.c.x, t.c.y, t.c.z );
  }
  glEnd();
  glPopAttrib();
  glPopMatrix();
}

void HapticTriangleSet::closestPoint( const Vec3 &p,
                                      Vec3 &cp,
                                      Vec3 &n,
                                      Vec3 &tc ) {
  Vec3 temp_cp, temp_n, temp_tc;
  Vec3 local_pos = transform.inverse() * p;
  HAPIFloat distance, temp_distance;
  for( unsigned int i = 0; i < triangles.size(); i++ ) {
    triangles[i].closestPoint( local_pos, temp_cp, temp_n, temp_tc );
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
