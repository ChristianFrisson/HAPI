 
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

#include <HAPI/HAPIHapticShape.h>
#include <OpenHapticsRenderer.h>
#include <HAPI/HAPIGLShape.h>
#include <HAPI/CollisionObjects.h>

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
    virtual void hlRender( HAPI::HAPIHapticsDevice *hd,
                           HLuint shape_id );
  protected:
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


    /// A upper bound on the number of triangles that will be rendered.
    /// Negative values will use the system default value.
    int nr_vertices;
    

    /// Enable HL_HAPTIC_CAMERA_VIEW or not
    bool use_haptic_camera;

    HAPIGLShape *gl_shape;
  };
}

#endif

#endif
