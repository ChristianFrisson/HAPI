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
/// \file OpenHapticsRenderer.h
/// \brief Header file for OpenHapticsRenderer.
///
//
//////////////////////////////////////////////////////////////////////////////


#ifndef __OPENHAPTICSRENDERER_H__
#define __OPENHAPTICSRENDERER_H__

#include <HAPI/Config.h>

#include <HAPI/HAPIProxyBasedRenderer.h>
#include <HAPI/HAPIHapticShape.h>
#include <H3DUtil/AutoPtrVector.h>
#include <H3DUtil/Threads.h>

#include <map>

#ifdef HAVE_OPENHAPTICS
#include <HL/hl.h>

#if defined(_MSC_VER) || defined(__BORLANDC__)
#pragma comment( lib, "hd.lib" )
//#pragma comment( lib, "hdu.lib" )
#endif

namespace HAPI {

  /// Haptics renderer using the HL API part of OpenHaptics for the 
  /// haptics rendering. 
  class OPENHAPTICSRENDERER_API OpenHapticsRenderer: 
    public HAPI::HAPIProxyBasedRenderer {
  public:

    /// Special shape type that allows for hl api specific rendering calls
    /// using the hlRender function.
    class OPENHAPTICSRENDERER_API HLShape {
    public:
      /// Destructor.
      virtual ~HLShape() {}
      /// This function performs all the HLAPI calls that are needed to render
      /// the surface. 
      virtual void hlRender( HAPI::HAPIHapticsDevice *hd,
                             HLuint shape_id ) = 0;
    };

    /// Special surface type that allows for hl api specific rendering calls
    /// using the hlRender function.
    class OPENHAPTICSRENDERER_API HLSurface {
    public:
      /// Destructor.
      virtual ~HLSurface() {}
      
      /// This function performs all the HLAPI calls that are needed to render
      /// the surface. 
      virtual void hlRender() = 0;
    };

    /// The OpenHapticsSurface is a HAPISurfaceObject that sets its parameters
    /// through OpenHaptics calls. It can only be used with OpenHapticsRenderer.
    class OPENHAPTICSRENDERER_API OpenHapticsSurface: public HAPISurfaceObject,
      public HLSurface
    {
    public:
      /// Constructor. 
      OpenHapticsSurface( HAPIFloat stiffness = 0.5,
                          HAPIFloat damping = 0,
                          HAPIFloat static_friction = 0.1,
                          HAPIFloat dynamic_friction = 0.4,
                          bool magnetic = false,
                          HAPIFloat snap_distance = 0.01,
                          bool use_ref_count_lock = true );

      /// Renders the surface using hlMaterialf calls
      virtual void hlRender();

      HAPIFloat stiffness, damping, static_friction,
                dynamic_friction, snap_distance;
      bool magnetic;
    };

    /// Options for what OpenHaptics parameters to use when rendering a shape.
    class OPENHAPTICSRENDERER_API OpenHapticsOptions: public HAPI::HAPIShapeRenderOptions {
    public:
      typedef enum {
        FEEDBACK_BUFFER,
        DEPTH_BUFFER,
        CUSTOM 
      } ShapeType;

      /// Constructor.
      OpenHapticsOptions( ShapeType _shape_type = FEEDBACK_BUFFER,
                          bool _use_adaptive_viewport = true,
                          bool _use_haptic_camera_view = true ):
        shape_type( _shape_type ),
        use_adaptive_viewport( _use_adaptive_viewport ),
        use_haptic_camera_view( _use_haptic_camera_view ) {
        
      }

      ShapeType shape_type;
      bool use_adaptive_viewport;
      bool use_haptic_camera_view;
      
    };

    typedef OpenHapticsOptions::ShapeType ShapeType;

    /// Initialize the renderer to be used with the given haptics device.
    virtual void initRenderer( HAPI::HAPIHapticsDevice *hd );

    /// Release all resources that has been used in the renderer for
    /// the given haptics device.
    virtual void releaseRenderer( HAPI::HAPIHapticsDevice *hd );

    /// Use HL API in OpenHaptics to render the shapes.
    virtual void preProcessShapes( HAPI::HAPIHapticsDevice *hd,
                                   const HapticShapeVector &shapes );
    /// Constructor.
    OpenHapticsRenderer( ShapeType _default_shape_type = 
                         OpenHapticsOptions::FEEDBACK_BUFFER,
                         bool _default_adaptive_viewport = true,
                         bool _default_haptic_camera_view = true ) :
      default_gl_shape( _default_shape_type ),
      default_adaptive_viewport( _default_adaptive_viewport ),
      default_haptic_camera_view( _default_haptic_camera_view ) {
        already_removed_id.reserve( 3 );
        dummy_context = NULL;
    }
                         
    
    /// Destructor.
    virtual ~OpenHapticsRenderer() {}
    
    /// The main function in any haptics renderer. Given a haptics device and 
    /// a group of shapes generate the force and torque to send to the device.
    /// Reads the forces generated by HL API and returns them.
    virtual HAPI::HAPIForceEffect::EffectOutput 
    renderHapticsOneStep( HAPI::HAPIHapticsDevice *hd,
                          const HapticShapeVector &shapes );

    /// Get the current proxy position.
    virtual HAPI::Vec3 getProxyPosition();

    /// Get the current default shape type for the renderer. 
    inline ShapeType getDefaultShapeType() {
      return default_gl_shape;
    }

    /// Set the current default shape type for the renderer.
    /// The default shape type is the shape type used if a shape
    /// has not defined a OpenHapticsOptions node.
    inline void setDefaultShapeType( ShapeType t ) {
      default_gl_shape = t;
    }

    /// Get the current default value for using adaptive viewport or not. 
    inline bool getDefaultAdaptiveViewport() {
      return default_adaptive_viewport;
    }

    /// Set the default value of if adaptive viewport should be used or
    /// not. The default value is used if a shape
    /// has not defined a OpenHapticsOptions node.
    inline void setDefaultAdaptiveViewport( bool t ) {
      default_adaptive_viewport = t;
    }

    /// Get the current default value for using haptic camera view
    /// or not. 
    inline bool getDefaultHapticCameraView() {
      return default_haptic_camera_view;
    }

    /// Set the default value of if haptic camera view should be used or
    /// not. The default value is used if a shape
    /// has not defined a OpenHapticsOptions node.
    inline void setDefaultHapticCameraView( bool t ) {
      default_haptic_camera_view = t;
    }


    /// Returns true if the given HAPISurfaceObject is supported to be 
    /// rendered by the OpenHapticsRenderer
    static bool surfaceSupported( HAPISurfaceObject *s );

    /// Renders a HAPISurface object with OpenHaptics. Returns true if it 
    /// succeeded, false otherwise. Not all surface types are valid.
    static bool hlRenderHAPISurface( HAPISurfaceObject *s );

    /// Sets up the surface parameters for HL API. All values are given in
    /// values between 0 and 1(except snapDistance which is in mm) just
    /// as normally done in OpenHaptics. If you want to specify absolute
    /// values instead use hlRenderAbsolute instead.
    static void hlRenderRelative( HAPIFloat stiffness,
                                  HAPIFloat damping,
                                  HAPIFloat static_friction,
                                  HAPIFloat dynamic_friction,
                                  bool magnetic = false,
                                  HAPIFloat snap_distance = 0 );

    /// Sets up the surface parameters for HL API. 
    /// TODO: Fix comment
    /// stiffness is given as N/mm
    /// damping as ...
    /// ..
    static void hlRenderAbsolute( HAPIFloat stiffness,
                                  HAPIFloat damping,
                                  HAPIFloat static_friction,
                                  HAPIFloat dynamic_friction,
                                  bool magnetic = false,
                                  HAPIFloat snap_distance = 0 );
    

    /// Register this renderer to the haptics renderer database.
    static HapticsRendererRegistration renderer_registration;

  protected:

    /// \internal
    class OPENHAPTICSRENDERER_API OpenHapticsWorkAroundToCleanUpHLContext :
    public WorkAroundToCleanUpHLContext {
    public:
      OpenHapticsWorkAroundToCleanUpHLContext() {
        dummy_context = NULL;
      }
      virtual void cleanUp();
      HHLRC dummy_context;
    };

    HHLRC dummy_context;

    ShapeType default_gl_shape;
    bool default_adaptive_viewport;
    bool default_haptic_camera_view;

    /// Callback function for finding the intersection between a line segment
    /// and the object. Used for custom shapes. 
    static HLboolean HLCALLBACK intersectCallback( 
                                      const HLdouble *start_point, 
                                      const HLdouble *end_point,
                                      HLdouble *intersection_point, 
                                      HLdouble *intersection_normal,
                                      HLenum* face,
                                      void *user_data );

    /// Callback function for finding the closest point on the object. Used
    /// in hlRender. Used for custom shapes.
    static HLboolean HLCALLBACK closestFeaturesCallback( 
                                       const HLdouble *query_point, 
                                       const HLdouble *target_point, 
                                       HLgeom *geom,
                                       HLdouble *closest_point,
                                       void* user_data );

    /// HL event callback function for when the geometry is touched.
    static void HLCALLBACK touchCallback( HLenum event,
                                          HLuint object,
                                          HLenum thread,
                                          HLcache *cache,
                                          void *userdata );

    /// HL event callback function for when the geometry is not touched
    /// any longer. 
    static void HLCALLBACK untouchCallback( HLenum event,
                                            HLuint object,
                                            HLenum thread,
                                            HLcache *cache,
                                            void *userdata );

    /// HL event callback function for when the proxy moves while in
    /// contact with the geometry.
    static void HLCALLBACK motionCallback( HLenum event,
                                           HLuint object,
                                           HLenum thread,
                                           HLcache *cache,
                                           void *userdata );

    struct CallbackData {
      CallbackData( OpenHapticsRenderer *r, 
                    HAPI::HAPIHapticShape *s,
                    HHLRC hc ): renderer( r ),
                                shape( s ),
                                haptic_context( hc ) {}
      OpenHapticsRenderer *renderer;
      H3DUtil::AutoRef< HAPI::HAPIHapticShape > shape;
      HHLRC haptic_context;
    };
    
    
    HLuint getHLShapeId( HAPI::HAPIHapticShape *hs,
                         HAPI::HAPIHapticsDevice *hd );

    H3DUtil::AutoPtrVector< CallbackData > callback_data; 

    /// Generate a new HL context for the given haptics device.
    HHLRC initHLLayer( HAPI::HAPIHapticsDevice *pd );

    typedef std::map< HAPI::HAPIHapticsDevice *, HHLRC > ContextMap;

    /// A map from haptics device to HL API context
    ContextMap context_map;

    /// A map from HAPI shape_id to HL API shape id
    typedef std::map< pair< int, HAPI::HAPIHapticsDevice * >, HLuint > IdMap;
    IdMap id_map;

    /// A map from HAPI shape_id to index in the callback_data vector
    typedef std::map< pair< int, HAPI::HAPIHapticsDevice * >, unsigned int >
      IdCbMap;
    IdCbMap id_cb_map;

    /// List used to manage adding and removing of event callbacks.
    typedef list< pair< int, HAPI::HAPIHapticsDevice * > > ShapeIdList;
    ShapeIdList previous_shape_ids;
    ShapeIdList previous_shape_ids_copy;

    /// Add HL_EVENT_MOTION, HL_EVENT_TOUCH and HL_EVENT_UNTOUCH callbacks
    /// to HL_CLIENT_THREAD.
    inline void addHLEventCallbacks( HLuint hl_id, CallbackData * cbd ) {
      hlEventd(  HL_EVENT_MOTION_LINEAR_TOLERANCE, 0 );
      hlAddEventCallback( HL_EVENT_MOTION, 
                          hl_id,
                          HL_CLIENT_THREAD,
                          &motionCallback,
                          cbd );
      hlAddEventCallback( HL_EVENT_TOUCH, 
                          hl_id,
                          HL_CLIENT_THREAD,
                          &touchCallback,
                          cbd );
      hlAddEventCallback( HL_EVENT_UNTOUCH, 
                          hl_id,
                          HL_CLIENT_THREAD,
                          &untouchCallback,
                          cbd );
    }

    /// Remove HL_EVENT_MOTION, HL_EVENT_TOUCH and HL_EVENT_UNTOUCH callbacks
    /// from HL_CLIENT_THREAD.
    inline void removeHLEventCallbacks( HLuint hl_id ) {
      hlRemoveEventCallback( HL_EVENT_MOTION, 
                             hl_id,
                             HL_CLIENT_THREAD,
                             &motionCallback );
      hlRemoveEventCallback( HL_EVENT_TOUCH, 
                             hl_id,
                             HL_CLIENT_THREAD,
                             &touchCallback );
      hlRemoveEventCallback( HL_EVENT_UNTOUCH, 
                             hl_id,
                             HL_CLIENT_THREAD,
                             &untouchCallback );
    }

    /// Used because of a supposed OpenHaptics Bug when untouchCallback is
    /// called before touchCallback for a specific shape.
    // TODO: confirm that it is indeed a OpenHaptics Bug by setting up a simple
    // example that gives the same result.
    vector< int > already_removed_id;
  };
}

#endif

#endif
