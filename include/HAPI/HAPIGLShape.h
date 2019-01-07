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
/// \file HAPIGLShape.h
/// \brief Header file for HAPIGLShape.h
///
//
//////////////////////////////////////////////////////////////////////////////
#ifndef __HAPIGLSHAPE_H__
#define __HAPIGLSHAPE_H__

#include <HAPI/HAPI.h>
#ifdef HAVE_OPENGL
#include <HAPI/HAPITypes.h>

namespace HAPI {

  /// \ingroup AbstractClasses
  /// \class HAPIGLShape
  /// \brief The base class for all classes which can be used with functions
  /// in the class FeedbackBufferCollector.
  ///
  /// The functions that HAPIGLShape can be used with are collectTriangles and
  /// collectPrimitives functions in the class FeedbackBufferCollector.
  class HAPI_API HAPIGLShape {
  public:
    /// Virtual destructor.
    virtual ~HAPIGLShape() {}

    /// Render the shape using OpenGL.
    virtual void glRender() {}

    /// A upper bound on how many triangles are renderered.
    virtual int nrTriangles() {
      return 0;
    }

    /// An upper bound on how many lines are renderered.
    virtual int nrLines() {
      return 0;
    }

    /// An upper bound on how many points are renderered.
    virtual int nrPoints() {
      return 0;
    }

    /// An axis aligned bounding box containing all the primitives rendered by 
    /// the glRender function. If no such bounding box is available, size
    /// should be set to Vec3( -1, -1, -1 )
    virtual void getBound( Vec3 &center, Vec3& size ) {
      center = Vec3( 0, 0, 0 );
      size = Vec3( -1, -1, -1 );
    }

    /// An upper bound on the number of vertices renderered.
    inline int nrVertices() {
      int nr_triangles = nrTriangles();
      int nr_lines = nrLines();
      int nr_points = nrPoints();
      if( nr_triangles == -1 || nr_lines == -1 || nr_points == - 1 )
        return -1;
      else
        return nr_triangles * 3 + nr_lines * 2 + nr_points;
    }

    /// An upper bound on the number of values required in the 
    /// feedbackbuffer in order to render it there.
    virtual int nrFeedbackBufferValues() {
      int nr_triangles = nrTriangles();
      int nr_lines = nrLines();
      int nr_points = nrPoints();
      if( nr_triangles == -1 || nr_lines == -1 || nr_points == - 1 )
        return -1;
      else
        // FeedbackBufferCollector requires 35 blocks for triangles.
        // 11 * 3 + 1 + 1 = 35 since this is how many times it reads the memory
        // for one triangle. Similar for lines is 11*2 + 1 = 23 and points = 
        // 11 + 1  = 12
        return nr_triangles * 35 + nr_lines * 23 + nr_points * 12;  
    }
  };
}
#endif //HAVE_OPENGL

#endif
