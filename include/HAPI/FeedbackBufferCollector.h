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
/// \file FeedbackBufferCollector.h
/// \brief Header file for FeedbackBufferCollector.
///
//
//////////////////////////////////////////////////////////////////////////////
#ifndef __FEEDBACKBUFFERCOLLECTOR_H__
#define __FEEDBACKBUFFERCOLLECTOR_H__

#include <HAPI/HAPIGLShape.h>
#include <HAPI/CollisionObjects.h>

#ifdef HAVE_OPENGL
#ifdef MACOSX
#include <OpenGL/gl.h>
#else
#include <GL/gl.h>
#endif

namespace HAPI {
  /// \ingroup Others
  /// \class FeedbackBufferCollector
  /// \brief Contains a bunch of static functions
  /// for collecting primitives rendered by OpenGL from the feedback buffer.
  ///
  /// If you have a class instance that inherit from HAPIGLShape you can
  /// use one of the collectTriangles() or collectPrimitives() functions.
  /// 
  /// If you just want to collect primitives from any OpenGL call you make
  /// you have to encapsulate them with \n
  /// startCollecting(..) \n
  /// . \n
  /// . \n
  /// endCollecting(..)\n
  /// 
  /// All calls in between startCollecting() and endCollecting() will not
  /// be rendered to the frame buffer, so if you want the same calls to be 
  /// rendered to the frame buffer as well, you will have to make them again
  /// outside the startCollecting() and endCollecting() pair.
  class HAPI_API FeedbackBufferCollector {
  public:
    
    /// Message generated when trying to collect OpenGL primitives.
    typedef enum {
      /// Success, no error occured
      SUCCESS,
      /// The feedback buffer did not have enough enough memory allocated
      NOT_ENOUGH_MEMORY_ALLOCATED,
      /// endCollecting() was called without a matching startCollecting() call
      END_COLLECT_WHEN_NOT_COLLECTING,
      /// startCollecting() was called while already collecting
      START_COLLECT_WHILE_COLLECTING
    } ErrorType;

    /// Collect all triangles rendered when calling the glRender() function for
    /// the given HAPIGLShape. 
    /// \param shape The HAPIGLShape which to collect triangles for.
    /// \param transform A transformation matrix to transform the coordinate 
    /// space before rendering the HAPIGLShape.
    /// \param triangles Return parameter. All triangles rendered are added 
    /// to this vector.
    /// \returns SUCCESS if every thing went well. An ErrorType with the error
    /// otherwise.
    static ErrorType collectTriangles( 
                            HAPIGLShape *shape,
                            const Matrix4 &transform,
                            std::vector< HAPI::Collision::Triangle > &triangles );

    /// Collect all primitives rendered when calling the glRender() function for
    /// the given HAPIGLShape. 
    /// \param _shape The HAPIGLShape which to collect primitives for.
    /// \param _transform A transformation matrix to transform the coordinate 
    /// space before rendering the HAPIGLShape.
    /// \param triangles Return parameter. All triangles rendered are added 
    /// to this vector.
    /// \param lines Return parameter. All line segments rendered are added 
    /// to this vector.
    /// \param points Return parameter. All points rendered are added 
    /// to this vector.
    /// \returns SUCCESS if every thing went well. An ErrorType with the error
    /// otherwise.
    static ErrorType collectPrimitives( 
                                HAPIGLShape *_shape,
                                const Matrix4 &_transform,
                                std::vector< HAPI::Collision::Triangle > &triangles,
                                std::vector< HAPI::Collision::LineSegment > &lines,
                                std::vector< HAPI::Collision::Point > &points );

    /// Start collecting OpenGL primitives from OpenGL calls. All OpenGL calls
    /// after running this function will not render anything to the frame buffer,
    /// but instead to the feedback buffer where all primitives will be recorded.
    /// Call one of the endCollecting() functions to get back to normal rendering
    /// and collect the rendered primitives.
    /// \param max_feedback_values The maximum number of values your OpenGL calls
    /// will need in the feedback buffer. 
    /// \param viewbox_center The center of a box within which primitives are 
    /// collected. All primitives outside the box will be ignored.
    /// \param viewbox_size The size of the box. If set to Vec3( -1, -1, -1 )
    /// the current projection matrix will be used.
    /// \returns SUCCESS if every thing went well. An ErrorType with the error
    /// otherwise.
    static ErrorType startCollecting( int max_feedback_values = -1,
                                Vec3 viewbox_center = Vec3(0, 0, 0 ),
                                Vec3 viewbox_size = Vec3( -1, -1, -1 ) );

    /// Get all the triangles rendered with OpenGL calls since the last call 
    /// of the startCollecting function. Also stops the collecting. Triangles
    /// will be added to the given vector. 
    /// \returns SUCCESS if every thing went well. An ErrorType with the error
    /// otherwise. If your GL calls have exceeded the max_feedback_values given 
    /// in startCollecting(), NOT_ENOUGH_MEMORY_ALLOCATED will be returned. 
    static ErrorType endCollecting( std::vector< HAPI::Collision::Triangle > &triangles );
    
    /// Get all the primitives rendered with OpenGL calls since the last call 
    /// of the startCollecting function. Also stops the collecting. Primitives
    /// will be added to the given vectors.
    /// \returns SUCCESS if every thing went well. An ErrorType with the error
    /// otherwise. If your GL calls have exceeded the max_feedback_values given 
    /// in startCollecting(), NOT_ENOUGH_MEMORY_ALLOCATED will be returned. 
    static ErrorType endCollecting( 
                               std::vector< HAPI::Collision::Triangle > &triangles,
                               std::vector< HAPI::Collision::LineSegment > &lines,
                               std::vector< HAPI::Collision::Point > &points );
    
  protected:
    static int parseVertex( GLfloat *buffer, int index, 
                            Vec3 &p, H3DUtil::RGBA &color, Vec3 &tc );
    static bool collecting_triangles;
    static GLfloat *buffer;
  };
}
#endif //HAVE_OPENGL
#endif
