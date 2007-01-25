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
/// \file HapticCylinder.cpp
/// \brief cpp file for HapticCylinder
///
//
//////////////////////////////////////////////////////////////////////////////

#include "HapticCylinder.h"

using namespace HAPI;
#ifdef HAVE_OPENjHAPTICS
/// Intersect the line segment from start_point to end_point with
/// the object.  
///
bool HapticCylinder::intersectSurface( const Vec3 &start_point, 
                                       const Vec3 &end_point,
                                       Vec3 &intersection_point, 
                                       Vec3 &intersection_normal,
                                       HLenum &face ) { 
  return false;
}

/// Find the closest point to query_point on the surface of the
/// object. 
/// 
bool HapticCylinder::closestFeature( const Vec3 &query_point, 
                                     const Vec3 &target_point,
                                     HLgeom *geom,
                                     Vec3 &closest_point ) {
  return false;
}

#include "GL/glew.h"
void HapticCylinder::hlRender( HLHapticsDevice *hd ) {
  HLSurface *s = dynamic_cast< HLSurface * >( surface );
  if( s ) {
    hlMatrixMode( HL_VIEWTOUCH );
    hlPushMatrix();
#if HL_VERSION_MAJOR_NUMBER >= 2
    hlPushAttrib( HL_MATERIAL_BIT | HL_TOUCH_BIT );
#endif
    const Matrix4 &m = transform;
    HLdouble vt[] = { m[0][0], m[1][0], m[2][0], 0,
                      m[0][1], m[1][1], m[2][1], 0,
                      m[0][2], m[1][2], m[2][2], 0,
                      m[0][3], m[1][3], m[2][3], 1 };
    hlLoadMatrixd( vt );
    s->hlRender( hd );
    hlTouchableFace( HL_FRONT_AND_BACK );
    hlRotatef( 90, 1, 0, 0 );
    hlTranslatef( 0, 0, -height/2 );
    hlBeginShape( HL_SHAPE_FEEDBACK_BUFFER, getShapeId( hd ) );
    GLUquadricObj *gl_quadric = gluNewQuadric();
    gluCylinder( gl_quadric, radius, radius, height, 50, 50 );
    gluDeleteQuadric( gl_quadric );
    hlEndShape();
#if HL_VERSION_MAJOR_NUMBER >= 2
     hlPopAttrib();
#endif
    hlPopMatrix();
  }
}
#endif

bool HapticCylinder::lineIntersect( const Vec3 &from, 
                           const Vec3 &to,
                           Bounds::IntersectionInfo &result ) {
  Matrix4 inverse = transform.inverse();
  Vec3 local_from = inverse * from;
  Vec3 local_to = inverse * to;
  Vec3 d = Vec3( 0, height, 0 );
  Vec3 n = local_to - local_from;
  HAPIFloat md = local_from * d;
  HAPIFloat nd = n * d;
  HAPIFloat dd = d * d;
  HAPIFloat t;
  
  // Test if segment fully outside either endcap of cylinder
  // Segment outside 'p' side of cylinder
  if( md < 0.0 && md + nd < 0.0 ) return false;
  // Segment outside 'q' side of cylinder
  if( md > dd && md + nd > dd ) return false;
  HAPIFloat nn = n * n;
  HAPIFloat mn = local_from * n;
  HAPIFloat a = dd * nn - nd * nd;
  HAPIFloat k = local_from * local_from - radius * radius;
  HAPIFloat c = dd * k - md * md;
  if( H3DUtil::H3DAbs(a) < Constants::epsilon ) {
    // Segment runs parallel to cylinder axis
    if( c > 0.0 ) return false; // 'a' and thus the segment lie outside cylinder
    // Now known that segment instersects cylinder; figure out how it intersects
    Vec3 normal;
    if( md < 0.0 ) {
      // Intersect segment against 'p' endcap
      t = -mn / nn;
      normal = Vec3( 0, -1, 0 );
    }
    else if( md > dd ) {
      // Intersect segment against 'q' endcap
      t = (nd - mn ) / nn;
      normal = Vec3( 0, 1, 0 );
    }
    else {
      t = 0.0; // 'a' lies inside cylinder
      normal = n / H3DUtil::H3DSqrt( nn );
    }

    result.point = transform * ( local_from + n * t );
    result.normal = transform.getRotationPart() * normal;
    return true;
  }
  HAPIFloat b = dd * mn - nd * md;
  HAPIFloat discr = b * b - a * c;
  if( discr < 0.0 ) return false; // No real roots; no intersection
  t = (-b - H3DUtil::H3DSqrt( discr ) ) / a;
  if( t < 0.0 || t > 1.0 ) return false; // Intersection lies outside segment
  if( md + t * nd < 0.0 ) {
    // Intersection outside cylinder on 'p' side
    if( nd <= 0.0 ) return false; // Segment pointing away from endcap
    t = -md / nd;
    // Keep intersection if Dot( S(t) -  p, S(t) - p ) < r^2
    if( k + 2 * t * (mn + t * nn ) <= 0.0 ) {
      result.point = transform * ( local_from + n * t );
      result.normal = transform.getRotationPart() * Vec3( 0, -1, 0 );
      return true;
    }
    return false;
  } else if( md + t * nd > dd ) {
    // Intersection outside cyilnder on 'q' side
    if( nd >= 0.0 ) return false; // Segment pointing away from endcap
    t = ( dd - md ) / nd;
    // Keep intersection if Dot( S(t) - q, S(t) - q ) <= r^2
    if( k + dd - 2 * md + t * (2 * (mn - nd ) + t * nn ) <= 0.0 ) {
      result.point = transform * ( local_from + n * t );
      result.normal = transform.getRotationPart() * Vec3( 0, 1, 0 );
      return true;
    }
    return false;
  }
  // Segment intersects cylinder between the endcaps; t is correct
  Vec3 local_point = local_from + n * t;
  result.point = transform * local_point;
  Vec3 local_normal = Vec3( local_point.x, 0, local_point.z  );
  local_normal.normalizeSafe();
  result.normal = transform.getRotationPart() * local_normal;
  return true;
}

void HapticCylinder::getConstraints( const Vec3 &point,
                            HAPIFloat radius,
                            std::vector< PlaneConstraint > &constraints ) {
  Vec3 cp, n, tc;
  closestPoint( point, cp, n, tc );
  constraints.push_back( PlaneConstraint( cp, n, tc, this ) );
}

void HapticCylinder::closestPoint( const Vec3 &p, Vec3 &cp, Vec3 &n, Vec3 &tc ) {
  // TODO: maybe remove the check if in between the end-caps and just choose one
  // of them in that case.
  Vec3 local_p = transform.inverse() * p;
  Vec3 normal_2d = Vec3( local_p.x, 0, local_p.z );
  HAPIFloat length = normal_2d.length();
  if( length > Constants::epsilon ) {
    if( length < radius ) {
      if( local_p.y <= 0.0 ) {
        local_p.y = 0.0;
        cp = transform * local_p;
        n = transform.getRotationPart() * Vec3( 0, -1, 0 );
      }
      else if( local_p.y >= height ) {
        local_p.y = height;
        cp = transform * local_p;
        n = transform.getRotationPart() * Vec3( 0, 1, 0 );
      }
      else {
        HAPIFloat b_cap_dist = H3DUtil::H3DAbs( local_p.y );
        HAPIFloat t_cap_dist = H3DUtil::H3DAbs( local_p.y - height );
        HAPIFloat t_rad_dis = radius - length;
        if( H3DUtil::H3DMin( b_cap_dist, t_cap_dist ) < t_rad_dis ) {
          if( H3DUtil::H3DAbs( b_cap_dist - t_cap_dist ) > Constants::epsilon ) {
            if( b_cap_dist < t_cap_dist ) {
              local_p.y = 0.0;
              cp = transform * local_p;
              n = transform.getRotationPart() * Vec3( 0, -1, 0 );
            } else {
              local_p.y = height;
              cp = transform * local_p;
              n = transform.getRotationPart() * Vec3( 0, 1, 0 );
            }
          }
          else {
            cerr << "two possible closest point, on either end_cap" << endl;
          }
        }
        else {
          normal_2d = normal_2d / length;
          cp =  radius * normal_2d;
          cp.y = local_p.y;
          cp = transform * cp;
          n = transform.getRotationPart() * normal_2d;
        }
      }
    } else {
      normal_2d = normal_2d / length;
      cp = radius * normal_2d;
      if( local_p.y <= 0.0 ) {
        cp = transform * cp;
        normal_2d = normal_2d + Vec3( 0, -1, 0 );
        normal_2d.normalize();
        n = transform.getRotationPart() * normal_2d;
      }
      else if( local_p.y >= height ) {
        cp.y = height;
        cp = transform * cp;
        normal_2d = normal_2d + Vec3( 0, 1, 0 );
        normal_2d.normalize();
        n = transform.getRotationPart() * normal_2d;
      }
      else {
        cp.y = local_p.y;
        cp = transform * cp;
        n = transform.getRotationPart() * normal_2d;
      }
    }
  } else {
    if( local_p.y <= 0.0 ) {
      cp = transform * Vec3( 0, 0, 0 );
      n = transform.getRotationPart() * Vec3( 0, -1, 0 );
    }
    else if( local_p.y >= height ) {
      cp = transform * Vec3( 0, height, 0 );
      n = transform.getRotationPart() * Vec3( 0, 1, 0 );
    }
    else {
      HAPIFloat b_cap_dist = H3DUtil::H3DAbs( local_p.y );
      HAPIFloat t_cap_dist = H3DUtil::H3DAbs( local_p.y - height );
      if( H3DUtil::H3DAbs( b_cap_dist - t_cap_dist ) > Constants::epsilon &&
          H3DUtil::H3DMin( b_cap_dist, t_cap_dist ) < radius ) {
        if( b_cap_dist < t_cap_dist ) {
          cp = transform * Vec3( 0, 0, 0 );
          n = transform.getRotationPart() * Vec3( 0, -1, 0 );
        } else {
          cp = transform * Vec3( 0, height, 0 );
          n = transform.getRotationPart() * Vec3( 0, 1, 0 );
        }
      }
      else {
        cerr << "infinite amount of closest points on the cylinder" << endl;
      }
    }
  }
}
