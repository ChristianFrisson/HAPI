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
/// \file HLDepthBufferShape.h
/// \brief Header file for HLDepthBufferShape
///
//
//////////////////////////////////////////////////////////////////////////////
#ifndef __HLDEPTHBUFFERSHAPE_H__
#define __HLDEPTHBUFFERSHAPE_H__

#include <HAPI/Config.h>

#include <HAPI/HAPIHapticShape.h>
#include <HAPI/OpenHapticsRenderer.h>

#ifdef HAVE_OPENHAPTICS
#ifdef HAVE_OPENGL

namespace HAPI {
  /// \ingroup OpenHapticsRenderer
  /// \ingroup Shapes
  /// \class HLDepthBufferShape
  /// \brief Class for using the glRender() function of an HAPIGLShape for
  /// haptics rendering. This makes it very easy to implement simple
  /// haptics rendering of a geometry, since when the glRender() function
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
  };

}

#endif //HAVE_OPENGL
#endif
#endif
