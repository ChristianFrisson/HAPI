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
/// \file FeedbackBufferCollectorExample.cpp
/// \brief CPP file which provides example code to show how to use the
/// FeedbackBufferCollector class of HAPI.
///
//
//////////////////////////////////////////////////////////////////////////////

// HAPI includes
#include <HAPI/FeedbackBufferCollector.h>
#ifdef HAVE_OPENGL
#include <HAPI/AnyHapticsDevice.h>
#include <HAPI/GodObjectRenderer.h>
#include <HAPI/FrictionSurface.h>
#include <HAPI/HapticTriangleTree.h>
#include <HAPI/HAPIProxyBasedRenderer.h>

// Other includes
#ifdef MACOSX
#include <OpenGL/gl.h>
#include <OpenGL/glu.h>
#include <GLUT/glut.h>
#else
#include <GL/gl.h>
#include <GL/glu.h>
#include <GL/glut.h>
#endif

#ifdef FREEGLUT
#include <GL/freeglut.h>
#endif

using namespace std;

// Pointer to the haptics device.
auto_ptr< HAPI::AnyHapticsDevice > hd(0);

// Render a sphere using OpenGL calls.
void draw() {
  GLUquadricObj* pObj = gluNewQuadric();
  gluSphere(pObj, 0.025, 10, 10);
  gluDeleteQuadric(pObj);
}

// Function to be called by GLUT to take care of rendering.
void display() {
  // Set up OpenGL state.
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
  glEnable(GL_DEPTH_TEST);
  glDepthFunc(GL_LESS);
  glDepthMask(GL_TRUE);
  glEnable(GL_CULL_FACE);
  glShadeModel(GL_SMOOTH);
  glEnable( GL_LIGHTING );
  glEnable(GL_LIGHT0);
  glMatrixMode( GL_MODELVIEW );
  glLoadIdentity();

  gluLookAt(0, 0, 0.2, 0, 0, 0, 0, 1, 0);
  glEnable( GL_NORMALIZE );

  // Draw the center sphere.
  draw();

  // Draw a primitive version of a stylus.
  if( hd.get() ) {
    HAPI::HAPIHapticsRenderer *hr = hd->getHapticsRenderer();
    if( hr ) {
      HAPI::Vec3 proxy_pos =
        static_cast< HAPI::HAPIProxyBasedRenderer * >(hr)->getProxyPosition();

      glPushMatrix();
      glTranslatef( (GLfloat)proxy_pos.x,
                    (GLfloat)proxy_pos.y,
                    (GLfloat)proxy_pos.z );
      glScalef( 0.1f, 0.1f, 0.1f );
      draw();
      glPopMatrix();
    }
  }

  // swap buffers.
  glutSwapBuffers();
}

// Function used as reshape callback by GLUT.
void reshape(GLint width, GLint height)
{
   glViewport(0, 0, width, height);
   glMatrixMode(GL_PROJECTION);
   glLoadIdentity();
   gluPerspective(65.0, (float)width / height, 0.01, 1000);
}

// Main.
int main(int argc, char* argv[]) {

  // Create a glut window
  glutInit(&argc, argv);
#ifdef FREEGLUT
  glutSetOption(GLUT_ACTION_ON_WINDOW_CLOSE, GLUT_ACTION_CONTINUE_EXECUTION);
#endif
  glutInitWindowSize( 640, 480 );
  glutInitDisplayMode( GLUT_RGBA | GLUT_DOUBLE | GLUT_DEPTH );
  int window_id = glutCreateWindow( "FeedbackBufferCollectorExample" );
  glutSetWindow( window_id );

  // set up GLUT callback functions
  glutDisplayFunc ( display  );
  glutReshapeFunc (reshape);
  glutIdleFunc( display );

  // container to put the triangles in.
  vector< HAPI::Collision::Triangle > triangles;

  // start collecting OpenGL primitives
  HAPI::FeedbackBufferCollector::startCollecting();

  // draw object
  draw();

  // stop collecting and transfer the rendered
  // triangles to the triangles vector.
  HAPI::FeedbackBufferCollector::endCollecting( triangles );

  // Get a connected device.
  hd.reset( new HAPI::AnyHapticsDevice() );

  // The haptics renderer to use.
  hd->setHapticsRenderer( new HAPI::GodObjectRenderer() );

  // Init the device.
  if( hd->initDevice() != HAPI::HAPIHapticsDevice::SUCCESS ) {
    cerr << hd->getLastErrorMsg() << endl;
    return 0;
  }

  // Enable the device
  hd->enableDevice();

  // Create the haptic shape to be rendered.
  HAPI::Collision::AABBTree *the_tree =
    new HAPI::Collision::AABBTree( triangles );
  HAPI::HAPISurfaceObject * my_surface =
    new HAPI::FrictionSurface();
  HAPI::HapticTriangleTree *tree_shape =
    new HAPI::HapticTriangleTree( the_tree, my_surface,
                                  HAPI::Collision::FRONT );

  // Transfer shape to the haptics loop.
  hd->addShape( tree_shape );
  hd->transferObjects();

  // Start gluts mainloop.
  glutMainLoop();

  // This will only be called if freeglut is found and used.
  hd->disableDevice();
  hd->releaseDevice();
}
#endif

