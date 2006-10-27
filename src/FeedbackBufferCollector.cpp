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
/// \file FeedbackBufferCollector.cpp
/// \brief CPP file for FeedbackBufferCollector.
///
//
//
//////////////////////////////////////////////////////////////////////////////

#include "FeedbackBufferCollector.h"

using namespace HAPI;

GLfloat *FeedbackBufferCollector::buffer = NULL;
bool FeedbackBufferCollector::collecting_triangles = false;

FeedbackBufferCollector::ErrorType 
FeedbackBufferCollector::collectTriangles( HAPIGLShape *shape,
                                           const Matrix4 &transform,
                                           vector< HAPI::Bounds::Triangle > &triangles ) {
  Vec3 center, size;
  shape->getBound( center, size );
  int nr_values = shape->nrFeedbackBufferValues();
  bool done = false;
  bool not_enough_memory =false;

  if( nr_values < 0 ) nr_values = 200000;

  while( !done ) {
    startCollecting( nr_values, center, size );
    glMatrixMode( GL_MODELVIEW );
    glPushMatrix();
    const Matrix4 &m = transform;
    GLdouble vt[] = { m[0][0], m[1][0], m[2][0], 0,
                      m[0][1], m[1][1], m[2][1], 0,
                      m[0][2], m[1][2], m[2][2], 0,
                      m[0][3], m[1][3], m[2][3], 1 };
    
    glLoadMatrixd( vt ); 
    shape->glRender();
    glMatrixMode( GL_MODELVIEW );
    glPopMatrix();
    ErrorType e = endCollecting( triangles );
    if( e != NOT_ENOUGH_MEMORY_ALLOCATED  ) done = true;  
    else {
      not_enough_memory = true;
      if( nr_values == 0 ) nr_values = 200000; 
      else nr_values *= 2;
    }
  }
  if( not_enough_memory ) return NOT_ENOUGH_MEMORY_ALLOCATED;
  else return SUCCESS;
}

FeedbackBufferCollector::ErrorType 
FeedbackBufferCollector::collectPrimitives( HAPIGLShape * shape,
                                            const Matrix4 &_transform,
                                            vector< HAPI::Bounds::Triangle > &triangles,
                                            vector< HAPI::Bounds::LineSegment > &lines,
                                            vector< HAPI::Bounds::Point > &points ) {
  Vec3 center, size;
  shape->getBound( center, size );

  int nr_values = shape->nrFeedbackBufferValues();
  bool done = false;
  bool not_enough_memory =false;
  
  if( nr_values < 0 ) nr_values = 200000;
  
  while( !done ) {
    startCollecting( nr_values, center, size );
    glMatrixMode( GL_MODELVIEW );
    glPushMatrix();
    const Matrix4 &m = _transform;
    GLdouble vt[] = { m[0][0], m[1][0], m[2][0], 0,
                      m[0][1], m[1][1], m[2][1], 0,
                      m[0][2], m[1][2], m[2][2], 0,
                      m[0][3], m[1][3], m[2][3], 1 };
    
    glLoadMatrixd( vt ); 
    shape->glRender();
    glMatrixMode( GL_MODELVIEW );
    glPopMatrix();
    ErrorType e = endCollecting( triangles, lines, points );
    if( e != NOT_ENOUGH_MEMORY_ALLOCATED  ) done = true;
    else {
      not_enough_memory = true;
      if( nr_values == 0 ) nr_values = 200000; 
      else nr_values *= 2;
    }
  }
  if( not_enough_memory ) return NOT_ENOUGH_MEMORY_ALLOCATED;
  else return SUCCESS;
}

FeedbackBufferCollector::ErrorType 
FeedbackBufferCollector::startCollecting( int max_feedback_values,
                                          Vec3 viewbox_center,
                                          Vec3 viewbox_size ) {
  if( collecting_triangles ) {
    return START_COLLECT_WHILE_COLLECTING;
  }

  collecting_triangles = true;
  glMatrixMode( GL_PROJECTION );
  glPushMatrix();

  if( max_feedback_values == -1 ) max_feedback_values = 3000000;
  buffer = new GLfloat[ max_feedback_values ];
  glFeedbackBuffer( max_feedback_values, GL_3D, buffer );

  glRenderMode( GL_FEEDBACK );
  if( viewbox_size.x != -1 && viewbox_size.y != -1 && viewbox_size.z != -1 ) {
    glLoadIdentity();
    Vec3 half_size = viewbox_size / 2;
    Vec3 llf = viewbox_center - half_size;
    Vec3 urn = viewbox_center + half_size;
    glOrtho( llf.x - 0.01, urn.x + 0.01, llf.y -0.01, urn.y + 0.01, -urn.z-0.01, -llf.z+0.01 );
    //glOrtho( -0.5, 0.5, -0.5, 0.5, 0, 10 );
  }
    
  glDisable(GL_CULL_FACE);
  glDisable(GL_LIGHTING); 
  return SUCCESS;
  
}

FeedbackBufferCollector::ErrorType 
FeedbackBufferCollector::endCollecting( 
           vector< HAPI::Bounds::Triangle > &triangles ) {
  if( !collecting_triangles ) return END_COLLECT_WHEN_NOT_COLLECTING;

  GLint nr_values = glRenderMode( GL_RENDER );

  if( nr_values == -1 ) {
    collecting_triangles = false;
    return NOT_ENOUGH_MEMORY_ALLOCATED;
  }

  GLdouble mv[16];
  glGetDoublev( GL_MODELVIEW_MATRIX, mv );
  
  GLdouble pm[16];
  glGetDoublev( GL_PROJECTION_MATRIX, pm );
  
  GLint vp[4];
  glGetIntegerv( GL_VIEWPORT, vp );

  Vec3 p;
  unsigned int vertex_index = 0;
  
  for( GLint i = 0; i < nr_values; ) {
    switch( (int)buffer[i++] ) {
    case( GL_POINT_TOKEN ): {
      i+= parseVertex( buffer, i, p );
      break;
    }
    case( GL_LINE_TOKEN ): {
      i+= parseVertex( buffer, i, p );
      i+= parseVertex( buffer, i, p );
      break;
    }
    case( GL_POLYGON_TOKEN ): {
      int nr_vertices =  (int)buffer[i];
      i++;
      Vec3 v0, v1, v2; 
      if( nr_vertices != 3 ) { 
        cerr << "Too Many vertices: " << nr_vertices << endl;
      }
      i+= parseVertex( buffer, i, p );
      gluUnProject( p.x, p.y, p.z, mv, pm, vp, &v0.x, &v0.y, &v0.z );
      i+= parseVertex( buffer, i, p );
      gluUnProject( p.x, p.y, p.z, mv, pm, vp, &v1.x, &v1.y, &v1.z );
      i+= parseVertex( buffer, i, p );
      gluUnProject( p.x, p.y, p.z, mv, pm, vp, &v2.x, &v2.y, &v2.z );
      triangles.push_back( HAPI::Bounds::Triangle( v0 *1e3, v1*1e3, v2*1e3 )) ;
      break;
    }
    case( GL_BITMAP_TOKEN ): 
    case( GL_DRAW_PIXEL_TOKEN ):
    case( GL_COPY_PIXEL_TOKEN ): {
      i+= parseVertex( buffer, i, Vec3() );
      break;
    }
    case( GL_PASS_THROUGH_TOKEN ): {
      i++;
      break;
    }
    };
  }
  glMatrixMode( GL_PROJECTION );
  glPopMatrix();
  delete buffer;
  collecting_triangles = false;
  return SUCCESS;
}

FeedbackBufferCollector::ErrorType 
FeedbackBufferCollector::endCollecting( 
           vector< HAPI::Bounds::Triangle > &triangles,
           vector< HAPI::Bounds::LineSegment > &lines,
           vector< HAPI::Bounds::Point > &points ) {
  if( !collecting_triangles ) return END_COLLECT_WHEN_NOT_COLLECTING;

  GLint nr_values = glRenderMode( GL_RENDER );
  if( nr_values == -1 ) {
    collecting_triangles = false;
    return NOT_ENOUGH_MEMORY_ALLOCATED;
  }

  GLdouble mv[16];
  glGetDoublev( GL_MODELVIEW_MATRIX, mv );
  
  GLdouble pm[16];
  glGetDoublev( GL_PROJECTION_MATRIX, pm );
  
  GLint vp[4];
  glGetIntegerv( GL_VIEWPORT, vp );

  Vec3 p;
  unsigned int vertex_index = 0;
  
  for( GLint i = 0; i < nr_values; ) {
    switch( (int)buffer[i++] ) {
    case( GL_POINT_TOKEN ): {
      i+= parseVertex( buffer, i, p );
      Vec3 pos;
      gluUnProject( p.x, p.y, p.z, mv, pm, vp, &pos.x, &pos.y, &pos.z );
      points.push_back( HAPI::Bounds::Point( pos ) );
      break;
    }
    case( GL_LINE_TOKEN ): {
      Vec3 v0, v1;
      i+= parseVertex( buffer, i, p );
      gluUnProject( p.x, p.y, p.z, mv, pm, vp, &v0.x, &v0.y, &v0.z );
      i+= parseVertex( buffer, i, p );
      gluUnProject( p.x, p.y, p.z, mv, pm, vp, &v1.x, &v1.y, &v1.z );
      lines.push_back( HAPI::Bounds::LineSegment( v0, v1 ) );
      break;
    }
    case( GL_POLYGON_TOKEN ): {
      int nr_vertices =  (int)buffer[i];
      i++;
      Vec3 v0, v1, v2; 
      if( nr_vertices != 3 ) { 
        cerr << "Too Many vertices: " << nr_vertices << endl;
      }
      i+= parseVertex( buffer, i, p );
      gluUnProject( p.x, p.y, p.z, mv, pm, vp, &v0.x, &v0.y, &v0.z );
      i+= parseVertex( buffer, i, p );
      gluUnProject( p.x, p.y, p.z, mv, pm, vp, &v1.x, &v1.y, &v1.z );
      i+= parseVertex( buffer, i, p );
      gluUnProject( p.x, p.y, p.z, mv, pm, vp, &v2.x, &v2.y, &v2.z );
      triangles.push_back( HAPI::Bounds::Triangle( v0 *1e3, v1*1e3, v2*1e3 )) ;
      break;
    }
    case( GL_BITMAP_TOKEN ): 
    case( GL_DRAW_PIXEL_TOKEN ):
    case( GL_COPY_PIXEL_TOKEN ): {
      i+= parseVertex( buffer, i, Vec3() );
      break;
    }
    case( GL_PASS_THROUGH_TOKEN ): {
      i++;
      break;
    }
    };
  } 
  glMatrixMode( GL_PROJECTION );
  glPopMatrix();
  delete buffer;
  collecting_triangles = false;
  return SUCCESS;
}


   
int FeedbackBufferCollector::parseVertex( GLfloat *buffer, int index, Vec3 &p ) {
  p = Vec3( buffer[index], buffer[index+1], buffer[index+2] );
  return 3;
}


