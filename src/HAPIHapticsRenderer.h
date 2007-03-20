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
/// \file HAPIHapticsRenderer.h
/// \brief Header file for HAPIHapticsRenderer.
///
//
//////////////////////////////////////////////////////////////////////////////

#ifndef __HAPIHAPTICSRENDERER_H__
#define __HAPIHAPTICSRENDERER_H__

#include <HAPI.h>
#include <HAPIHapticShape.h>
#include <HAPIForceEffect.h>
#include <AutoRefVector.h>
#include <Threads.h>
#include <list>
#include <map>

namespace HAPI {

  // forward declaration
  class HAPIHapticsDevice;

  /// \class HAPIHapticsRenderer
  /// \brief Base class for all haptics renderers in HAPI.
  /// The job of a haptics renderer is to given a set of geometries and the
  /// current values of a haptics device generate a force and a torque to send
  /// to the haptics device to render. Subclasses must define the following function
  /// that does just that:
  /// -   HapticForceEffect::EffectOutput 
  ///     renderHapticsOneStep( HAPIHapticsDevice *hd, 
  ///                           const HapticShapeVector &shapes )
  ///
  /// Other functions that can be optionally overridden and does not do anything 
  /// by default are:
  /// - void initRenderer( HAPIHapticsDevice *hd ) - initialize the renderer for use
  /// with a haptics device. Allocate resources needed for doing the rendering.
  /// - void releaseRenderer( HAPIHapticsDevice *hd ) - release all resources allocated
  /// in initRenderer.
  /// - void preProcessShapes( HAPIHapticsDevice *hd,
  ///                          const HapticShapeVector &shapes ) - will be called
  /// in the main loop when new shapes are transferred to be rendered in the 
  /// haptics loop. It can be used to make some last changes to the shapes before
  /// moving them over to the haptics loop.
  ///
  /// There exists a database over all available haptics renderers. All subclasses 
  /// should contain a database entry as:
  /// static HapticsRendererRegistration renderer_registration;
  /// See for example RuspiniRenderer.h/cpp for an example. 
  class HAPI_API HAPIHapticsRenderer {
  public:

    typedef H3DUtil::AutoRefVector< HAPIHapticShape > HapticShapeVector;

    /// Initialize the renderer to be used with the given haptics device.
    virtual void initRenderer( HAPIHapticsDevice *hd ) {}

    /// Release all resources that has been used in the renderer for
    /// the given haptics device.
    virtual void releaseRenderer( HAPIHapticsDevice *hd ) {}


    /// This function will be called in the main loop when new shapes are
    /// transferred to be rendered in the haptics loop. It can be used
    /// to make some last changes to the shapes before moving them over
    /// to the haptics loop.
    virtual void preProcessShapes( HAPIHapticsDevice *hd,
                                   const HapticShapeVector &shapes ) {} 

    /// Destructor. Stops haptics rendering and remove callback functions.
    virtual ~HAPIHapticsRenderer();

    /// The main function in any haptics renderer. Given a haptics device and 
    /// a group of shapes generate the force and torque to send to the device.
    virtual HAPIForceEffect::EffectOutput 
    renderHapticsOneStep( HAPIHapticsDevice *hd,
                          const HapticShapeVector &shapes ) {
      return HAPIForceEffect::EffectOutput();
    };

    typedef std::vector< pair< H3DUtil::AutoRef< HAPI::HAPIHapticShape >,
                               HAPISurfaceObject::ContactInfo> > Contacts; 

    inline Contacts getContacts() {
      contacts_lock.lock();
      Contacts c = contacts;
      contacts_lock.unlock();
      return c;
    }

    // The following is part of the database of available haptics renderers.
    typedef HAPIHapticsRenderer*( *CreateInstanceFunc)(); 

    template< class N >
    static HAPIHapticsRenderer *newInstance() { return new N; };

    /// Class used to register a class to the registered file readers.
    struct HAPI_API HapticsRendererRegistration{
    public:
      /// Constructor.
      HapticsRendererRegistration( const string &_name,
                                   CreateInstanceFunc _create ):
      name( _name ),
      create_func( _create ) {
        
        if( !HAPIHapticsRenderer::initialized ) {
          HAPIHapticsRenderer::registered_renderers = 
            new list< HapticsRendererRegistration >;
          initialized = true;
        }
        HAPIHapticsRenderer::registerRenderer( *this );
      }

      string name;
      CreateInstanceFunc create_func;
    };
#ifdef __BORLANDC__
    friend struct HapticsRendererRegistration;
#endif

    /// Register a haptics renderer to the database.
    /// \param name The name of the renderer
    /// \param create A function for creating an instance of that class.
    /// \param supports A function to determine if the class supports a
    /// given file type.
    static void registerRenderer( const string &name,
                                  CreateInstanceFunc create ) {
      registerRenderer( HapticsRendererRegistration( name, create ) );
    }

    /// Register a haptics renderer that can then be returned by 
    /// getSupportedFileReader().
    static void registerRenderer( const HapticsRendererRegistration &fr ) {
      registered_renderers->push_back( fr );
    }

    // solution to clean up dummy_context in OpenHapticsRenderer correctly.
    // Could perhaps switch for just calling hdstartscheduler in
    // releaserenderer for OpenHapticsRenderer.
    void cleanUpStuff( HAPIHapticsDevice *hd );

  protected:
    static list< HapticsRendererRegistration > *registered_renderers;
    H3DUtil::MutexLock contacts_lock;
    Contacts contacts;
    static bool initialized;

    class HAPI_API WorkAroundToCleanUpHLContext {
    public:
      virtual void cleanUp() {}
    };

    static map< HAPIHapticsDevice *,
      vector< WorkAroundToCleanUpHLContext * > > clean_up_stuff;
  };
}

#endif
