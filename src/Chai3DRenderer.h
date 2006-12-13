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
/// \file Chai3DRenderer.h
/// \brief Header file for Chai3DRenderer.
///
//
//////////////////////////////////////////////////////////////////////////////

#ifndef __CHAI3DRENDERER_H__
#define __CHAI3DRENDERER_H__

// H3DApi includes
#include <H3DApi.h>

// HAPI includes
#include <HAPIProxyBasedRenderer.h>
#include <HAPIHapticShape.h>

// H3DUtil includes
#include <AutoPtrVector.h>
#include <Threads.h>

// Chai3D includes
#include <CGenericDevice.h>
#include <CGeneric3dofPointer.h>
#include <CWorld.h>

#if defined(_MSC_VER) || defined(__BORLANDC__)
#pragma comment( lib, "chai3d_complete.lib" )
#endif

namespace HAPI {

  /// \class Chai3DRenderer
  /// \brief Haptics renderer using the HL API part of Chai3D for the 
  /// haptics rendering.
  class H3DAPI_API Chai3DRenderer: public HAPI::HAPIProxyBasedRenderer {
  public:
    class H3DAPI_API Chai3DShape {
    public:
      virtual ~Chai3DShape() {}
      /// This function performs all the Chai3DAPI calls that are needed
      /// to render the surface. 
      virtual void chai3dRender( HAPI::HAPIHapticsDevice *hd ) = 0;
    };

    class H3DAPI_API Chai3DSurface {
    public:
      /// Destructor.
      virtual ~Chai3DSurface() {}
      
      /// Sets a Chai3D cMaterial describing the haptic properties for the 
      /// surface.
      virtual void chai3dMaterial( cMaterial &m ) = 0;
    };


    class H3DDevice: public cGenericDevice {
    public:
      // CONSTRUCTOR & DESTRUCTOR:
      //! Constructor of cPhantomDevice.
      H3DDevice() {
        m_systemReady = true;
        m_systemAvailable = true;
      }
      //! Destructor of cPhantomDevices.
      ~H3DDevice() {}
      
      // METHODS:
      //! Open connection to phantom device.
      virtual int open() { return 0; }
      //! Close connection to phantom device.
      virtual int close() { return 0; }
      //! Calibrate phantom device.
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
      
      //! Ask the device to call me back periodically
      virtual bool setCallback(cCallback* m_callback) { return false; }

    protected:
      Vec3 position, velocity;
      Rotation orientation;
      int buttons;

    };

    class H3DTool : public cGeneric3dofPointer {
    public:
      // CONSTRUCTOR & DESTRUCTOR:
      //! Constructor of cPhantom3dofPointer.
      H3DTool(cWorld* a_world) : cGeneric3dofPointer( a_world ) {
        m_device = new H3DDevice;
      }
    };

    class H3DAPI_API HLSurface {
    public:
      /// Destructor.
      virtual ~HLSurface() {}
      
      /// This function performs all the HLAPI calls that are needed to render
      /// the surface. 
      virtual void hlRender() = 0;
    };

    class H3DAPI_API Chai3DOptions: public HAPI::HAPIShapeRenderOptions {
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

    /// Set the radius of the proxy(in millimetres)
    inline void setProxyRadius( HAPIFloat r ) {
      cProxyPointForceAlgo *p = chai3d_tool->getProxy();
      if( p ) p->setProxyRadius( r );
    }

    /// Initialize the renderer to be used with the given haptics device.
    virtual void initRenderer( HAPI::HAPIHapticsDevice *hd );

    /// Release all resources that has been used in the renderer for
    /// the given haptics device.
    virtual void releaseRenderer( HAPI::HAPIHapticsDevice *hd );

    /// Use HL API in Chai3D to render the shapes.
    virtual void preProcessShapes( HAPI::HAPIHapticsDevice *hd,
                                   const HapticShapeVector &shapes );
    /// Constructor.
    Chai3DRenderer( )  {
      world = new cWorld;
      chai3d_tool.reset( new H3DTool( world ) );
      world->addChild(chai3d_tool.get());
      chai3d_tool->useNormalizedPositions( false );
    }
    
    /// Destructor.
    virtual ~Chai3DRenderer() {}
    
    /// The main function in any haptics renderer. Given a haptics device and 
    /// a group of shapes generate the force and torque to send to the device.
    /// Reads the forces generated by HL API and returns them.
    virtual HAPI::HapticForceEffect::EffectOutput 
    renderHapticsOneStep( HAPI::HAPIHapticsDevice *hd,
                          const HapticShapeVector &shapes );

    /// Get the current proxy position.
    virtual HAPI::Vec3 getProxyPosition();

    /// Register this renderer to the haptics renderer database.
    static HapticsRendererRegistration renderer_registration;

  protected:
    auto_ptr< H3DTool > chai3d_tool;
    cWorld *world;
    H3DUtil::AutoPtrVector< cMesh > meshes; 
    MutexLock mesh_change_lock;
  };
}

#endif
