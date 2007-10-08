
//////////////////////////////////////////////////////////////////////////////
//    H3D API.   Copyright 2004, Daniel Evestedt, Mark Dixon
//    All Rights Reserved
//
/// \file HLDepthBufferShape.h
/// \brief Header file for HLDepthBufferShape.
///
//
//////////////////////////////////////////////////////////////////////////////
#ifndef __HLDEPTHBUFFERSHAPE_H__
#define __HLDEPTHBUFFERSHAPE_H__

#include <Config.h>

#include <HAPIHapticShape.h>
#include <OpenHapticsRenderer.h>

#ifdef HAVE_OPENHAPTICS

namespace HAPI {
  /// Class for using the render() function of an X3DGeometryNode for
  /// haptics rendering. This makes it very easy to implement simple
  /// haptics rendering of a geometry, since when the render() function
  /// is implemented to perform the graphical rendering. The shape is
  /// implemented as a HL_SHAPE_DEPTH_BUFFER shape in HLAPI.
  class OPENHAPTICSRENDERER_API HLDepthBufferShape: public HAPIHapticShape, 
                                     public OpenHapticsRenderer::HLShape  {
  public:
    /// Constructor.
    HLDepthBufferShape( HAPIGLShape *_glshape,
                        const Matrix4 &_transform,
                        HAPISurfaceObject *_surface,
                        Collision::FaceType _touchable_face = 
                        Collision::FRONT_AND_BACK,
                        bool _use_haptic_camera = true,
                        bool _use_adaptive_viewport = true,
                        void *_userdata = NULL,
                        int _shape_id = -1,
                        void (*_clean_up_func)( void * ) = 0 ):
      HAPIHapticShape( _transform, _surface, _touchable_face, _userdata,
                       _shape_id, _clean_up_func ),
      use_haptic_camera( _use_haptic_camera ),
      use_adaptive_viewport( _use_adaptive_viewport ),
      gl_shape( _glshape ) {}
    
   virtual bool lineIntersectShape( const Vec3 &from, 
                                     const Vec3 &to,
                                     Collision::IntersectionInfo &result,
                                     Collision::FaceType face = 
                                     Collision::FRONT_AND_BACK  ) {
      return false;
    }
    
    virtual void getConstraintsOfShape( const Vec3 &point,
                                        Constraints &constraints,
                                        Collision::FaceType face = 
                                        Collision::FRONT_AND_BACK ,
                                        HAPIFloat radius = -1 ) {}

    virtual void closestPointOnShape( const Vec3 &p, Vec3 &cp, 
                                      Vec3 &n, Vec3 &tc ) {}

    virtual bool movingSphereIntersectShape( HAPIFloat radius,
                                             const Vec3 &from, 
                                             const Vec3 &to ) {
      return false;
    }

    virtual void glRenderShape() {}


    /// This function performs all the HLAPI calls that are needed to render
    /// the shape. Uses HL_SHAPE_FEEDBACK_BUFFER to render the object.     
    virtual void hlRender( HAPI::HAPIHapticsDevice *hd,
                           HLuint id );

    /// Which sides of the faces are touchable.
    Collision::FaceType touchable_face;
    
    /// Enable HL_HAPTIC_CAMERA_VIEW or not
    bool use_haptic_camera;

   /// Enable HL_ADAPTIVE_VIEWPORT or not
    bool use_adaptive_viewport;

    HAPIGLShape *gl_shape;
  };

}

#endif
#endif
