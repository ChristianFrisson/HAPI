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
/// \file HAPIGLShape.h
/// \brief Header file for HAPIGLShape.h
///
//
//////////////////////////////////////////////////////////////////////////////
#ifndef __HAPIGLSHAPE_H__
#define __HAPIGLSHAPE_H__

#include <HAPI.h>

namespace HAPI {

  /// The base class for all classes which can be used with the 
  /// getFeedbackBufferTriangles functions.
  class HAPI_API HAPIGLShape {
  public:
    /// Render the shape using OpenGL.
    virtual void glRender() {}

    /// A lower bound on how many triangles are renderered.
    virtual int nrTriangles() {
      return 0;
    }

    /// An upperr bound on how many lines are renderered.
    virtual int nrLines() {
      return 0;
    }

    /// An upper bound on how many lines are renderered.
    virtual int nrPoints() {
      return 0;
    }

    /// An upper bound on the number of vertices renderered.
    inline int nrVertices() {
      return nrTriangles() * 3 + nrLines() * 2 + nrPoints();
    }

    /// An upper bound on the number of values required in the 
    /// feedbackbuffer in order to render it there.
    virtual int nrFeedbackBufferValues() {
      return nrTriangles() * 11 + nrLines() * 7 + nrPoints() * 4;  
    }
  };
}

#endif
