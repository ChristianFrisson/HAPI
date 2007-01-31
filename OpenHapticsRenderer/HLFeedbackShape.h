 
//////////////////////////////////////////////////////////////////////////////
//    H3D API.   Copyright 2004, Daniel Evestedt, Mark Dixon
//    All Rights Reserved
//
/// \file HLFeedbackShape.h
/// \brief Header file for HLFeedbackShape.
///
//
//////////////////////////////////////////////////////////////////////////////
#ifndef __HLFEEDBACKSHAPE_H__
#define __HLFEEDBACKSHAPE_H__

#include <Config.h>

#include <HAPIHapticShape.h>
#include <OpenHapticsRenderer.h>
#include <HAPIGLShape.h>
#include <CollisionObjects.h>

#ifdef HAVE_OPENHAPTICS

namespace HAPI {

  /// Class for using the render() function of an X3DGeometryNode for
  /// haptics rendering. This makes it very easy to implement simple
  /// haptics rendering of a geometry, since when the render() function
  /// is implemented to perform the graphical rendering. The shape is
  /// implemented as a HL_SHAPE_FEEDBACK_BUFFER shape in HLAPI.
  class OPENHAPTICSRENDERER_API HLFeedbackShape: public HAPIHapticShape, 
                                                 public OpenHapticsRenderer::HLShape  {
  public:
    //    typedef int HLint;
    typedef Bounds::FaceType FaceType;

    /// Constructor.
    HLFeedbackShape( HAPIGLShape *_glshape,
                     HAPI::HAPISurfaceObject *_surface,
                     const Matrix4 &_transform,
                     int _nr_vertices = -1,
                     FaceType _touchable_face = Bounds::FRONT_AND_BACK,
                     bool _use_haptic_camera = true ):
      HAPI::HAPIHapticShape( _glshape, _surface, _transform ),
      nr_vertices( _nr_vertices ),
      touchable_face( _touchable_face ),
      use_haptic_camera( _use_haptic_camera ) {}
    
    /// This function performs all the HLAPI calls that are needed to render
    /// the shape. Uses HL_SHAPE_FEEDBACK_BUFFER to render the object.     
    virtual void hlRender( HAPI::HAPIHapticsDevice *hd,
                           HLuint shape_id );
  protected:
    /// A upper bound on the number of triangles that will be rendered.
    /// Negative values will use the system default value.
    int nr_vertices;
    
    /// Which sides of the faces are touchable.
    FaceType touchable_face;
    
    /// Enable HL_HAPTIC_CAMERA_VIEW or not
    bool use_haptic_camera;
  };
}

#endif

#endif