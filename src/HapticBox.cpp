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
/// \file HapticBox.cpp
/// \brief cpp file for HapticBox
///
//
//////////////////////////////////////////////////////////////////////////////
#include "HapticBox.h"

using namespace HAPI;
#ifdef HjAVE_OPENHAPTICS
/// Intersect the line segment from start_point to end_point with
/// the object.  
///
bool HapticBox::intersectSurface( const Vec3 &start_point, 
                                     const Vec3 &end_point,
                                     Vec3 &intersection_point, 
                                     Vec3 &intersection_normal,
                                     HLenum &face ) { 
  // p is the starting point of the ray used for determinining intersection
  // v is the vector between this starting point and the end point.
  // the starting point must be outside the sphere.
  Vec3 p, v;
  HAPIFloat r2 = radius * radius;
  face = HL_FRONT;

  HAPIFloat a0  = start_point * start_point - r2;
  if (a0 <= 0) {
    // start_point is inside sphere
    a0 = end_point * end_point - r2;
    if( a0 <= 0 ) {
      // both start_point and end_point are inside sphere so no intersection
      return false;
    } else {
      // start_point is inside and end_point is outside. We will have an 
      // intersection.
      p = end_point;
      v = start_point - end_point;
      face = HL_BACK;
    }
  } else {
    // start_point is outside sphere
    p = start_point;
    v = end_point - start_point;
    
    // check that the line will intersect 
    HAPIFloat a1 = v * p;
    if (a1 >= 0) {
      // v is pointing away from the sphere so no intersection
      return false;
    }
  }
  // use implicit quadratic formula to find the roots
  HAPIFloat a = v.x*v.x + v.y*v.y + v.z*v.z;
  HAPIFloat b = 2 * (p.x*v.x + p.y*v.y + p.z*v.z);
  HAPIFloat c = p.x*p.x + p.y*p.y + p.z*p.z - r2;

  HAPIFloat s = b*b - 4*a*c;

  HAPIFloat u;
  if( s == 0.0 ) {
    // line is a tangent to the sphere
    u = -b/(2*a);
  } else if( s > 0.0 ) {
    // line intersects sphere in two points
    HAPIFloat u0 = (-b + sqrt(s))/(2*a);
    HAPIFloat u1 = (-b - sqrt(s))/(2*a);
    u = u0 < u1 ? u0 : u1;
  }  else {
    // line does not intersect
    return false;
  }

  // check that the intersection point is within the line segment
  if( u > 1.0 || u < 0.0) {
    return false;
  }
  
  // calculate the intersection point and normal
  intersection_point = p + u*v;
  intersection_normal = face == HL_FRONT ? 
    intersection_point: -intersection_point;
  intersection_normal.normalize();
  return true;
}

/// Find the closest point to query_point on the surface of the
/// object. 
/// 
bool HapticBox::closestFeature( const Vec3 &query_point, 
                                   const Vec3 &target_point,
                                   HLgeom *geom,
                                   Vec3 &closest_point ) {
  Vec3 closest_normal = query_point;
  closest_normal.normalize();
  closest_point = closest_normal * radius;
  
  HLdouble cn[] = { closest_normal.x, 
                    closest_normal.y,
                    closest_normal.z };
  HLdouble cp[] = { closest_point.x, 
                    closest_point.y,
                    closest_point.z };
#if !defined(__APPLE__)
  hlLocalFeature2dv( geom, HL_LOCAL_FEATURE_PLANE, 
                     cn, cp );
#endif  
  return true;
}

void HapticBox::hlRender( HLHapticsDevice *hd) {
  HLSurface *s = dynamic_cast< HLSurface * >( surface );
  if( s ) {
     glMatrixMode( GL_MODELVIEW );
     glPushMatrix();
#if HL_VERSION_MAJOR_NUMBER >= 2
     hlPushAttrib( HL_MATERIAL_BIT | HL_TOUCH_BIT );
#endif
     const Matrix4 &m = transform;
     GLfloat vt[] = { m[0][0], m[1][0], m[2][0], 0,
                      m[0][1], m[1][1], m[2][1], 0,
                      m[0][2], m[1][2], m[2][2], 0,
                      m[0][3], m[1][3], m[2][3], 1 };
     glLoadMatrixf( vt );
     s->hlRender( hd );
     if( solid ) 
       hlTouchableFace( HL_FRONT );
     else
       hlTouchableFace( HL_FRONT_AND_BACK ); 
     HLCustomObject::hlRender( hd );
#if HL_VERSION_MAJOR_NUMBER >= 2
     hlPopAttrib();
#endif
     glPopMatrix();
  }
}
#endif // HAVE_OPENHAPTICS

bool HapticBox::lineIntersect( const Vec3 &start_point, 
                                  const Vec3 &end_point,
                                  Bounds::IntersectionInfo &result ) {
  HAPIFloat tmin = 0.0f;
  HAPIFloat tmax = 1.0f;
  Matrix4 inverse = transform.inverse();
  Vec3 local_end_point = inverse * end_point;
  Vec3 local_start_point = inverse * start_point;
  Vec3 d = local_end_point - local_start_point;
  // For all three slabs
  for( int i = 0; i < 3; i++ ) {
    if( H3DUtil::H3DAbs(d[i]) < Constants::epsilon ) {
      // Ray is parallel to slab. No hit if origin not within slab;
      if( start_point[i] < min[i] || start_point[i] > max[i] ) return false;
    } else {
      // Compute intersection t value of ray with near and far plane of slab
      HAPIFloat ood = 1.0 / d[i];
      HAPIFloat t1 = (min[i] - start_point[i]) * ood;
      HAPIFloat t2 = (max[i] - start_point[i]) * ood;
      // Make t1 be intersection with near plane, t2 with far plane
      if( t1 > t2 ) {
        HAPIFloat temp = t1;
        t1 = t2;
        t2 = temp;
      }
      // Compute the intersection of slab intersection intervals
      if( t1 > tmin ) tmin = t1;
      if( t2 > tmax ) tmax = t2;
      // Exit with no collision as soon as slab intersection becomes empty
      if( tmin > tmax ) return false;
    }
  }
  // Ray intersect all 3 slabs. Return point and intersection t value (tmin)
  Vec3 temp_point = start_point + d * tmin;
  // TODO: Normal, should be 22 different cases. (inside outside too)
  // is this failsafe and fast? the box is assumed to not have 0 size in any of
  // the directions
  Vec3 normal;
  for( int i = 0; i < 3; i++ ) {
    if( H3DUtil::H3DAbs(temp_point[i] - min[i]) <= Constants::epsilon )
      normal[i] = normal[i] - 1;
    else if( H3DUtil::H3DAbs( temp_point[i] - max[i]) <= Constants::epsilon )
      normal[i] = normal[i] + 1;
  }
  normal.normalizeSafe();

  result.point = transform * temp_point;
  result.normal = transform.getRotationPart() * normal;
  return true;
}


void HapticBox::getConstraints(  const Vec3 &point,
                                    HAPIFloat r,
                                    std::vector< PlaneConstraint > &result ) {
  Vec3 cp, n, tc;
  closestPoint( point, cp, n, tc );
  result.push_back( PlaneConstraint( cp, n, tc, this ) );
}

void HapticBox::closestPoint( const Vec3 &p, Vec3 &cp, Vec3 &n, Vec3 &tc ) { 
  Vec3 temp_p = transform.inverse() * p;
  
  bool inside[3] = {false, false, false };

  for( int i = 0; i < 3; i++ ) {
    HAPIFloat v = temp_p[i];
    if( v < min[i] ) v = min[i];
    else if( v > max[i] ) v = max[i];
    else inside[i] = true;
    cp[i] = v;
  }

  if( temp_p.x > min.x && temp_p.x < max.x &&
      temp_p.y > min.y && temp_p.y < max.y &&
      temp_p.z > min.z && temp_p.z < max.z ) {
    
    Vec3 closest = temp_p;
    closest.x = min.x;
    HAPIFloat dist = temp_p.x - min.x;
    
    HAPIFloat temp_dist = temp_p.y - min.y;
    if( temp_dist < dist ) {
      closest.x = temp_p.x;
      closest.y = min.y;
      dist = temp_dist;
    }

    temp_dist = temp_p.z - min.z;
    if( temp_dist < dist ) {
      closest = temp_p;
      closest.z = min.z;
      dist = temp_dist;
    }

    temp_dist = max.x - temp_p.x;
    if( temp_dist < dist ) {
      closest = temp_p;
      closest.x = max.x;
      dist = temp_dist;
    }

    temp_dist = max.y - temp_p.y;
    if( temp_dist < dist ) {
      closest = temp_p;
      closest.y = max.y;
      dist = temp_dist;
    }

    temp_dist = max.z - temp_p.z;
    if( temp_dist < dist ) {
      closest = temp_p;
      closest.z = max.z;
      dist = temp_dist;
    }

    cp = closest;
  }

  for( int i = 0; i < 3; i++ ) {
    if( H3DUtil::H3DAbs(cp[i] - min[i]) <= Constants::epsilon )
      n[i] = n[i] - 1;
    else if( H3DUtil::H3DAbs( cp[i] - max[i]) <= Constants::epsilon )
      n[i] = n[i] + 1;
  }
  n.normalizeSafe();

  cp = transform * cp;
  n = transform.getRotationPart() * cp;
}
