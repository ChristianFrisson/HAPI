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
/// \file HLFeedbackShape.h
/// \brief Header file for HLFeedbackShape
///
//
//////////////////////////////////////////////////////////////////////////////
#ifndef __HLFEEDBACKSHAPE_H__
#define __HLFEEDBACKSHAPE_H__

#include <HAPI/Config.h>

#include <HAPI/HAPIHapticShape.h>
#ifdef HAVE_OPENGL
#include <HAPI/OpenHapticsRenderer.h>
#include <HAPI/HAPIGLShape.h>
#include <HAPI/CollisionObjects.h>

#ifdef HAVE_OPENHAPTICS

namespace HAPI {

  /// \ingroup OpenHapticsRenderer
  /// \ingroup Shapes
  /// \class HLFeedbackShape
  /// \brief Class for using the glRender() function of an HAPIGLShape for
  /// haptics rendering. This makes it very easy to implement simple
  /// haptics rendering of a geometry, since when the glRender() function
  /// is implemented to perform the graphical rendering. The shape is
  /// implemented as a HL_SHAPE_FEEDBACK_BUFFER shape in HLAPI.
  class OPENHAPTICSRENDERER_API HLFeedbackShape: public HAPIHapticShape, 
                                                 public OpenHapticsRenderer::HLShape  {
  public:
    //    typedef int HLint;
    typedef Collision::FaceType FaceType;

    /// Constructor.
    HLFeedbackShape( HAPIGLShape *_glshape,
                     const Matrix4 &_transform,
                     HAPISurfaceObject *_surface,
                     Collision::FaceType _touchable_face = 
                     Collision::FRONT_AND_BACK,
                     bool _use_haptic_camera = true,
                     bool _use_adaptive_viewport = true,
                     int _nr_vertices = -1,
                     void *_userdata = NULL,
                     int _shape_id = -1,
                     void (*_clean_up_func)( void * ) = 0 ):
      HAPI::HAPIHapticShape( _transform, _surface, _touchable_face, _userdata,
                             _shape_id, _clean_up_func  ),
      nr_vertices( _nr_vertices ),
      use_haptic_camera( _use_haptic_camera ),
      gl_shape( _glshape) {}
    
    /// This function performs all the HLAPI calls that are needed to render
    /// the shape. Uses HL_SHAPE_FEEDBACK_BUFFER to render the object.
    /// \param hd The haptics device for which the surface should be rendered.
    /// \param _shape_id The HL-id for the shape.
    virtual void hlRender( HAPI::HAPIHapticsDevice *hd,
                           HLuint _shape_id );
  protected:
    /// Must be defined in order to link correctly.
    virtual bool lineIntersectShape( const Vec3 &from, 
                                     const Vec3 &to,
                                     Collision::IntersectionInfo &result,
                                     Collision::FaceType face = 
                                     Collision::FRONT_AND_BACK  ) {
      return false;
    }
    
    /// Must be defined in order to link correctly.
    virtual void getConstraintsOfShape( const Vec3 &point,
                                        Constraints &constraints,
                                        Collision::FaceType face = 
                                        Collision::FRONT_AND_BACK ,
                                        HAPIFloat radius = -1 ) {}

    /// Must be defined in order to link correctly.
    virtual void closestPointOnShape( const Vec3 &p, Vec3 &cp, 
                                      Vec3 &n, Vec3 &tc ) {}

    /// Must be defined in order to link correctly.
    virtual bool movingSphereIntersectShape( HAPIFloat radius,
                                             const Vec3 &from, 
                                             const Vec3 &to ) {
      return false;
    }

    /// Must be defined in order to link correctly.
    virtual void getTangentSpaceMatrixShape( const Vec3 &point,
                                             Matrix4 &result_mtx ) {}

    /// Must be defined in order to link correctly.
    virtual void glRenderShape() {}


    /// A upper bound on the number of triangles that will be rendered.
    /// Negative values will use the system default value.
    int nr_vertices;
    

    /// Enable HL_HAPTIC_CAMERA_VIEW or not
    bool use_haptic_camera;

    HAPIGLShape *gl_shape;
  };
}

#endif

#endif //HAVE_OPENGL

#endif
