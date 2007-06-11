
//////////////////////////////////////////////////////////////////////////////
//    H3D API.   Copyright 2004, Daniel Evestedt, Mark Dixon
//    All Rights Reserved
//
/// \file FeedbackBufferCollector.h
/// \brief Header file for FeedbackBufferCollector.
///
//
//////////////////////////////////////////////////////////////////////////////
#ifndef __FEEDBACKBUFFERCOLLECTOR_H__
#define __FEEDBACKBUFFERCOLLECTOR_H__

#include <HAPIGLShape.h>
#include <CollisionObjects.h>

namespace HAPI {
  /// The FeedbackBufferCollector contains a bunch of static functions for 
  /// collecting primitives rendered by OpenGL from the feedback buffer.
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
    static ErrorType collectTriangles( HAPIGLShape *shape,
                                       const Matrix4 &transform,
                                       vector< HAPI::Bounds::Triangle > &triangles );

    /// Collect all primitives rendered when calling the glRender() function for
    /// the given HAPIGLShape. 
    /// \param shape The HAPIGLShape which to collect primitives for.
    /// \param transform A transformation matrix to transform the coordinate 
    /// space before rendering the HAPIGLShape.
    /// \param triangles Return parameter. All triangles rendered are added 
    /// to this vector.
    /// \param lines Return parameter. All line segments rendered are added 
    /// to this vector.
    /// \param points Return parameter. All points rendered are added 
    /// to this vector.
    /// \returns SUCCESS if every thing went well. An ErrorType with the error
    /// otherwise.
    static ErrorType collectPrimitives( HAPIGLShape *_shape,
                                        const Matrix4 &_transform,
                                        vector< HAPI::Bounds::Triangle > &triangles,
                                        vector< HAPI::Bounds::LineSegment > &lines,
                                        vector< HAPI::Bounds::Point > &points );

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
    static ErrorType endCollecting( vector< HAPI::Bounds::Triangle > &triangles );
    
    /// Get all the primitives rendered with OpenGL calls since the last call 
    /// of the startCollecting function. Also stops the collecting. Primitives
    /// will be added to the given vector.
    /// \returns SUCCESS if every thing went well. An ErrorType with the error
    /// otherwise. If your GL calls have exceeded the max_feedback_values given 
    /// in startCollecting(), NOT_ENOUGH_MEMORY_ALLOCATED will be returned. 
    static ErrorType endCollecting( vector< HAPI::Bounds::Triangle > &triangles,
                                    vector< HAPI::Bounds::LineSegment > &lines,
                                    vector< HAPI::Bounds::Point > &points );
    
  protected:
    static int parseVertex( GLfloat *buffer, int index, 
                            Vec3 &p, H3DUtil::RGBA &color, Vec3 &tc );
    static bool collecting_triangles;
    static GLfloat *buffer;
  };
}

#endif
