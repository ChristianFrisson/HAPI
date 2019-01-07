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
/// \file Chai3DRenderer.h
/// \brief Header file for Chai3DRenderer.
///
//
//////////////////////////////////////////////////////////////////////////////


#ifndef __CHAI3DRENDERER_H__
#define __CHAI3DRENDERER_H__

// HAPI includes
#include <HAPI/HAPIProxyBasedRenderer.h>
#include <HAPI/HAPIHapticShape.h>

#include <HAPI/Chai3DRendererConfig.h>

// H3DUtil includes
#include <H3DUtil/AutoPtrVector.h>
#include <H3DUtil/Threads.h>

#ifdef HAVE_CHAI3D

// Chai3D includes
#ifdef CHAI3D_VERSION_2_0
#ifndef _MSVC
#define _MSVC
#endif
#include <devices/CGenericDevice.h>
#include <tools/CGeneric3dofPointer.h>
#include <scenegraph/CWorld.h>
#else
#include <CGenericDevice.h>
#include <CGeneric3dofPointer.h>
#include <CWorld.h>
#endif

namespace HAPI {

  /// \defgroup Chai3DRenderer Chai3DRenderer classes
  /// These classes use HAPI but use Chai3D for geometry based haptic
  /// rendering and are built in a project separate from HAPI.
  /// The reason for this is to separate dependency on dlls in Windows.

  /// \ingroup Chai3DRenderer
  /// \ingroup Renderers
  /// \class Chai3DRenderer
  /// \brief Haptics renderer using Chai3D(www.chai3d.org) for the
  /// geometry based haptics rendering.
  ///
  /// This is one of several geometry based haptics renderers that can be used
  /// in HAPI.
  class CHAI3DRENDERER_API Chai3DRenderer: public HAPIProxyBasedRenderer {
  public:

    /// \ingroup Chai3DRenderer
    /// \class Chai3DShape
    /// \brief Base class for special shape types that uses Chai3D calls to
    /// render the shape.
    class CHAI3DRENDERER_API Chai3DShape {
    public:
      virtual ~Chai3DShape() {}
      /// This function performs all the Chai3DAPI calls that are needed
      /// to render the shape. 
      /// \param hd The haptics device on which the Chai3DShape should be
      /// rendered.
      virtual void chai3dRender( HAPIHapticsDevice *hd ) = 0;
    };

    /// \ingroup Chai3DRenderer
    /// \class Chai3DSurface
    /// \brief Base class for special shape types that uses Chai3D calls to
    /// render the surface.
    class CHAI3DRENDERER_API Chai3DSurface {
    public:
      /// Destructor.
      virtual ~Chai3DSurface() {}
      
      /// Sets a Chai3D cMaterial describing the haptic properties for the 
      /// surface.
      virtual void chai3dMaterial( cMaterial &m ) = 0;
    };

    /// \ingroup Chai3DRenderer
    /// \class H3DDevice
    /// \brief Our implementation of cGenericDevice which must be subclassed.
    class H3DDevice: public
#ifdef CHAI3D_VERSION_2_0
      cGenericHapticDevice {
#else
      cGenericDevice {
#endif
    public:
      /// Constructor.
      H3DDevice() {
        m_systemReady = true;
        m_systemAvailable = true;
      }

      /// Destructor.
      ~H3DDevice() {}
      
      virtual int open() { return 0; }

      virtual int close() { return 0; }

      virtual int initialize(const bool a_resetEncoders=false) {
        return 0;
      }

      void setNewValues( Vec3 pos,
                         Vec3 vel,
                         Rotation orn,
                         int but ) {
        position = pos;
        velocity = vel;
        orientation = orn;
        buttons = but;
      }

      //! Send a command to the phantom device.
      virtual int command(int a_command, void* a_data);

#ifdef CHAI3D_VERSION_2_0
      virtual int getPosition( cVector3d &a_position );

      virtual int getLinearVelocity( cVector3d &a_linearVelocity );

      virtual int getRotation( cMatrix3d &a_rotation );

      virtual int getUserSwitch( int a_switchIndex, bool &a_status );
#endif

      //! Ask the device to call me back periodically
      virtual bool setCallback(cCallback* m_callback) { return false; }

    protected:
      Vec3 position, velocity;
      Rotation orientation;
      int buttons;

    };

    /// \ingroup Chai3DRenderer
    /// \class H3DTool
    /// \brief Our subclass of cGeneric3dofPointer of Chai3D.
    class H3DTool : public cGeneric3dofPointer {
    public:
      H3DTool(cWorld* a_world) : cGeneric3dofPointer( a_world ) {
        // If default cGenericdofPointer constructor creates a device. Delete
        // it.
        if( m_device )
          delete m_device;
        m_device = new H3DDevice;
      }

      ~H3DTool() {
        // Clean up memory, might not be needed in future version of Chai3D
        // if cGeneric3dofPointer will clean up its dynamically allocated
        // functions by itself.
#ifdef CHAI3D_VERSION_2_0
        if( m_proxyPointForceModel ) {
          delete m_proxyPointForceModel;
          m_proxyPointForceModel = NULL;
        }

        if( m_potentialFieldsForceModel ) {
          delete m_potentialFieldsForceModel;
          m_potentialFieldsForceModel = NULL;
        }
#else
        for( unsigned int i = 0; i < m_pointForceAlgos.size(); ++i ) {
          delete m_pointForceAlgos[i];
        }
        m_pointForceAlgos.clear();
#endif
      }

#ifdef CHAI3D_VERSION_2_0
      inline cGenericHapticDevice * getDevice() {
        return m_device;
      }
#endif
    };

    /// \ingroup Chai3DRenderer
    /// \class Chai3DOptions
    /// \brief Used to set specific options for Chai3D. No options are Chai3D
    /// specific at the moment.
    class CHAI3DRENDERER_API Chai3DOptions: public HAPIShapeRenderOptions {
    public:

      Chai3DOptions() {   
      }
    };


    /// Get the current radius of the proxy.
    inline HAPIFloat getProxyRadius() {
      cProxyPointForceAlgo *p = chai3d_tool->getProxy();
      if( p ) return p->getProxyRadius();
      else return 0;
    }

    /// Set the radius of the proxy.
    inline void setProxyRadius( HAPIFloat r ) {
      cProxyPointForceAlgo *p = chai3d_tool->getProxy();
      if( p ) p->setProxyRadius( r );
    }

    /// Initialize the renderer to be used with the given haptics device.
    virtual void initRenderer( HAPIHapticsDevice *hd );

    /// Release all resources that has been used in the renderer for
    /// the given haptics device.
    virtual void releaseRenderer( HAPIHapticsDevice *hd );

    /// Use HL API in Chai3D to render the shapes.
    virtual void preProcessShapes( HAPIHapticsDevice *hd,
                                   const HapticShapeVector &shapes );
    /// Constructor.
    Chai3DRenderer( )  {
      world = new cWorld;
      chai3d_tool.reset( new H3DTool( world ) );
#ifndef CHAI3D_VERSION_2_0
      chai3d_tool->useNormalizedPositions( false );
#endif
    }
    
    /// Destructor.
    virtual ~Chai3DRenderer() {
      // This code is needed before cleaning up because otherwise both
      // H3DAPI and HAPI will try to delete the meshes that are children
      // to the world. This assumes that preProcessShapes and
      // renderHapticsOneStep are no longer called for this renderer.
      world->clearAllChildren();
      meshes.clear();
      // Cleanup
      delete world;
      world = NULL;
    }
    
    /// The main function in any haptics renderer. Given a haptics device and 
    /// a group of shapes generate the force and torque to send to the device.
    /// Reads the forces generated by HL API and returns them.
    virtual HAPIForceEffect::EffectOutput 
    renderHapticsOneStep( HAPIHapticsDevice *hd,
                          const HapticShapeVector &shapes );

    /// Get the current proxy position.
    virtual Vec3 getProxyPosition();

    /// Register this renderer to the haptics renderer database.
    static HapticsRendererRegistration renderer_registration;

  protected:
    std::auto_ptr< H3DTool > chai3d_tool;
    cWorld *world;
    H3DUtil::AutoPtrVector< cMesh > meshes; 
    H3DUtil::MutexLock mesh_change_lock;
    std::map< cMesh *, HAPIHapticShape * > mesh_map;
  };
}

#endif //HAVE_CHAI3D

#endif


