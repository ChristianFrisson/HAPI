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
/// \file HapticSphere.cpp
/// \brief cpp file for HapticSphere
///
//
//////////////////////////////////////////////////////////////////////////////
#include "HapticSphere.h"

using namespace H3D;
#ifdef HAVE_OPENHAPTICS
/// Intersect the line segment from start_point to end_point with
/// the object.  
///
bool HapticSphere::intersectSurface( const Vec3f &start_point, 
                                     const Vec3f &end_point,
                                     Vec3f &intersection_point, 
                                     Vec3f &intersection_normal,
                                     HLenum &face ) { 
  // p is the starting point of the ray used for determinining intersection
  // v is the vector between this starting point and the end point.
  // the starting point must be outside the sphere.
  Vec3f p, v;
  H3DFloat r2 = radius * radius;
  face = HL_FRONT;

  H3DFloat a0  = start_point * start_point - r2;
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
    H3DFloat a1 = v * p;
    if (a1 >= 0) {
      // v is pointing away from the sphere so no intersection
      return false;
    }
  }
  // use implicit quadratic formula to find the roots
  H3DFloat a = v.x*v.x + v.y*v.y + v.z*v.z;
  H3DFloat b = 2 * (p.x*v.x + p.y*v.y + p.z*v.z);
  H3DFloat c = p.x*p.x + p.y*p.y + p.z*p.z - r2;

  H3DFloat s = b*b - 4*a*c;

  H3DFloat u;
  if( s == 0.0 ) {
    // line is a tangent to the sphere
    u = -b/(2*a);
  } else if( s > 0.0 ) {
    // line intersects sphere in two points
    H3DFloat u0 = (-b + sqrt(s))/(2*a);
    H3DFloat u1 = (-b - sqrt(s))/(2*a);
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
bool HapticSphere::closestFeature( const Vec3f &query_point, 
                                   const Vec3f &target_point,
                                   HLgeom *geom,
                                   Vec3f &closest_point ) {
  Vec3f closest_normal = query_point;
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

void HapticSphere::hlRender( HLHapticsDevice *hd) {
  HLSurface *s = dynamic_cast< HLSurface * >( surface );
  if( s ) {
     hlMatrixMode( HL_VIEWTOUCH );
     hlPushMatrix();
#if HL_VERSION_MAJOR_NUMBER >= 2
     hlPushAttrib( HL_MATERIAL_BIT | HL_TOUCH_BIT );
#endif
     const Matrix4f &m = transform;
     HLfloat vt[] = { m[0][0], m[1][0], m[2][0], 0,
                      m[0][1], m[1][1], m[2][1], 0,
                      m[0][2], m[1][2], m[2][2], 0,
                      m[0][3], m[1][3], m[2][3], 1 };
     hlLoadMatrixf( vt );
     s->hlRender( hd );
     if( solid ) 
       hlTouchableFace( HL_FRONT );
     else
       hlTouchableFace( HL_FRONT_AND_BACK ); 
     HLCustomObject::hlRender( hd );
#if HL_VERSION_MAJOR_NUMBER >= 2
     hlPopAttrib();
#endif
     hlPopMatrix();
  }
}
#endif

bool HapticSphere::lineIntersect( const Vec3d &start_point, 
                                  const Vec3d &end_point,
                                  Bounds::IntersectionInfo &result ) {
  // p is the starting point of the ray used for determinining intersection
  // v is the vector between this starting point and the end point.
  // the starting point must be outside the sphere.
  Vec3d p, v;
  H3DDouble r2 = radius * radius;

  H3DDouble a0  = start_point * start_point - r2;
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
    }
  } else {
    // start_point is outside sphere
    p = start_point;
    v = end_point - start_point;
    
    // check that the line will intersect 
    H3DDouble a1 = v * p;
    if (a1 >= 0) {
      // v is pointing away from the sphere so no intersection
      return false;
    }
  }
  // use implicit quadratic formula to find the roots
  H3DDouble a = v.x*v.x + v.y*v.y + v.z*v.z;
  H3DDouble b = 2 * (p.x*v.x + p.y*v.y + p.z*v.z);
  H3DDouble c = p.x*p.x + p.y*p.y + p.z*p.z - r2;

  H3DDouble s = b*b - 4*a*c;

  H3DDouble u;
  if( s == 0.0 ) {
    // line is a tangent to the sphere
    u = -b/(2*a);
  } else if( s > 0.0 ) {
    // line intersects sphere in two points
    H3DDouble u0 = (-b + sqrt(s))/(2*a);
    H3DDouble u1 = (-b - sqrt(s))/(2*a);
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
  result.point = p + u*v;
  result.normal = result.point;
  result.normal.normalize();
  return true;
}


void HapticSphere::getConstraints(  const Vec3d &point,
                                    H3DDouble r,
                                    std::vector< PlaneConstraint > &result ) {

  Vec3d v = point;
  H3DDouble v2 = v.lengthSqr();
  
  if(  //v2 <= r * r && 
       v2 > Constants::d_epsilon ) {
    v = v / H3DSqrt( v2 );
    result.push_back( PlaneConstraint( (radius+0.0025) * v, v ) );
  } else {
    cerr << point << endl;
  }
}
