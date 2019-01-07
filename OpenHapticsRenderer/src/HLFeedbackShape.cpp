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
/// \file HLFeedbackShape.cpp
/// \brief cpp file for HLFeedbackShape
///
//
//////////////////////////////////////////////////////////////////////////////

#include <HAPI/HLFeedbackShape.h>


#ifdef HAVE_OPENHAPTICS

#ifdef HAVE_OPENGL
#ifdef MACOSX
#include <OpenGL/gl.h>
#else
#include <GL/gl.h>
#endif

using namespace HAPI;

void HLFeedbackShape::hlRender( HAPI::HAPIHapticsDevice *hd,
                                HLuint hl_shape_id ) {

  if( OpenHapticsRenderer::surfaceSupported( surface.get() ) ) {

    //&& closeEnoughToBound( hd->proxyPosition->getValue(), 
    //hd->getPreviousProxyPosition(), 
    //(Matrix4f) transform.inverse(), 
    //geometry ) ) {
    glMatrixMode( GL_MODELVIEW );
    glPushMatrix();
#if HL_VERSION_MAJOR_NUMBER >= 2
    hlPushAttrib( HL_MATERIAL_BIT | HL_TOUCH_BIT );
#endif
    const Matrix4 &m = transform;
    GLdouble vt[] = { m[0][0], m[1][0], m[2][0], 0,
                      m[0][1], m[1][1], m[2][1], 0,
                      m[0][2], m[1][2], m[2][2], 0,
                      m[0][3], m[1][3], m[2][3], 1 };
    glLoadIdentity();
    glScalef( 1e3f, 1e3f, 1e3f ); 
    glMultMatrixd( vt );

    GLboolean use_hl_modelview = !hlIsEnabled( HL_USE_GL_MODELVIEW );
    if( use_hl_modelview ) {
      // Set HL_MODELVIEW if that one should be used instead of GL_MODELVIEW.
      HLdouble hvt[] = { vt[0], vt[1], vt[2], vt[3],
                         vt[4], vt[5], vt[6], vt[7],
                         vt[8], vt[9], vt[10], vt[11],
                         vt[12], vt[13], vt[14], vt[15] };
      hlMatrixMode( HL_MODELVIEW );
      hlPushMatrix();
      hlLoadIdentity();
      hlScaled( 1e3, 1e3, 1e3 );
      hlMultMatrixd( hvt );
    }

    OpenHapticsRenderer::hlRenderHAPISurface( surface.get(), hd );
    if( touchable_face == Collision::BACK ) hlTouchableFace( HL_BACK );
    else if( touchable_face == Collision::FRONT ) hlTouchableFace( HL_FRONT );
    else if( touchable_face == Collision::FRONT_AND_BACK )
      hlTouchableFace( HL_FRONT_AND_BACK );

    if( use_haptic_camera )
      hlEnable( HL_HAPTIC_CAMERA_VIEW );
    else
      hlDisable( HL_HAPTIC_CAMERA_VIEW );

    Matrix3 m3 = m.getScaleRotationPart();
    GLint front_face;

    bool negative_scaling = 
      ( ( m3.getRow( 0 ) % m3.getRow( 1 ) ) * m3.getRow(2) ) < 0;
    glGetIntegerv( GL_FRONT_FACE, &front_face );
    // if front_face == GL_CW then the GLWindow we render to is mirrored.
    // front_face has to be flipped each time a negative scaling is applied
    // in order for normals to be correct. 
    if( front_face == GL_CW ) {
      if( !negative_scaling )
        glFrontFace( GL_CCW );
    } else if( negative_scaling )
      glFrontFace( GL_CW );

    if( nr_vertices >= 0 ) {
      hlHinti( HL_SHAPE_FEEDBACK_BUFFER_VERTICES, nr_vertices );
    } else {
      hlHinti(  HL_SHAPE_FEEDBACK_BUFFER_VERTICES, 65536 );
    }

    hlBeginShape( HL_SHAPE_FEEDBACK_BUFFER, hl_shape_id );
    gl_shape->glRender();
    hlEndShape();
    
    glFrontFace( front_face );
#if HL_VERSION_MAJOR_NUMBER >= 2
    hlPopAttrib();
#endif
    glMatrixMode( GL_MODELVIEW );
    glPopMatrix();
    if( use_hl_modelview ) {
      hlMatrixMode( HL_MODELVIEW );
      hlPopMatrix();
    }
  }
}

#endif //HAVE_OPENGL
#endif

