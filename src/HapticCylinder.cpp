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

using namespace H3D;

#ifdef HAVE_OPENHAPTICS
/// Intersect the line segment from start_point to end_point with
/// the object.  
///
bool HapticCylinder::intersectSurface( const Vec3f &start_point, 
                                       const Vec3f &end_point,
                                       Vec3f &intersection_point, 
                                       Vec3f &intersection_normal,
                                       HLenum &face ) { 
  return false;
}

/// Find the closest point to query_point on the surface of the
/// object. 
/// 
bool HapticCylinder::closestFeature( const Vec3f &query_point, 
                                     const Vec3f &target_point,
                                     HLgeom *geom,
                                     Vec3f &closest_point ) {
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
    const Matrix4f &m = transform;
    HLfloat vt[] = { m[0][0], m[1][0], m[2][0], 0,
                     m[0][1], m[1][1], m[2][1], 0,
                     m[0][2], m[1][2], m[2][2], 0,
                     m[0][3], m[1][3], m[2][3], 1 };
    hlLoadMatrixf( vt );
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
