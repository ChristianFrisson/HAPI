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
/// \file HAPIHapticsRenderer.h
/// \brief Header file for HAPIHapticsRenderer.
///
//
//////////////////////////////////////////////////////////////////////////////

#ifndef __HAPIHAPTICSRENDERER_H__
#define __HAPIHAPTICSRENDERER_H__

#include <HAPI/HAPI.h>
#include <HAPI/HAPIHapticShape.h>
#include <HAPI/HAPIForceEffect.h>
#include <H3DUtil/AutoRefVector.h>
#include <H3DUtil/Threads.h>
#include <list>
#include <map>

namespace HAPI {

  // forward declaration
  class HAPIHapticsDevice;

  /// \ingroup AbstractClasses
  /// \class HAPIHapticsRenderer
  /// \brief Base class for all haptics renderers in HAPI.
  ///
  /// The purpose of a haptics renderer is, given a set of geometries and the
  /// current values of a haptics device, to generate a force and a torque to
  /// send to the haptics device to render. Subclasses must define the
  /// following function that does this: 
  /// -   HapticForceEffect::EffectOutput 
  ///     renderHapticsOneStep( HAPIHapticsDevice *hd, 
  ///                           const HapticShapeVector &shapes,
  ///                           H3DTime dt )
  ///                          
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

    /// This function is kept for backwards compatibility with older haptic
    /// renderers that do not accept dt.
    virtual HAPIForceEffect::EffectOutput 
      renderHapticsOneStep( HAPIHapticsDevice *hd,
                            const HapticShapeVector &shapes ) {
      return HAPIForceEffect::EffectOutput();
    };

    /// \brief The main function in any haptics renderer. Given a haptics
    /// device and a group of shapes generate the force and torque to send
    /// to the device. dt is the timestep from the last haptic loop to current.
    virtual HAPIForceEffect::EffectOutput 
      renderHapticsOneStep( HAPIHapticsDevice *hd,
                            const HapticShapeVector &shapes,
                            HAPITime dt ) {
      return renderHapticsOneStep( hd, shapes );
    };

    typedef std::vector< std::pair< H3DUtil::AutoRef< HAPI::HAPIHapticShape >,
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
    static HAPIHapticsRenderer *newInstance() { return new N; }

    /// Struct used to register a class to the registered renderers.
    struct HAPI_API HapticsRendererRegistration{
    public:
      /// Constructor.
      HapticsRendererRegistration( const std::string &_name,
                                   CreateInstanceFunc _create ):
      name( _name ),
      create_func( _create ) {
        
        if( !HAPIHapticsRenderer::initialized ) {
          HAPIHapticsRenderer::registered_renderers.reset( 
            new std::list< HapticsRendererRegistration > );
          initialized = true;
        }
        HAPIHapticsRenderer::registerRenderer( *this );
      }

      std::string name;
      CreateInstanceFunc create_func;
    };
#ifdef __BORLANDC__
    friend struct HapticsRendererRegistration;
#endif

    /// Register a haptics renderer to the database.
    /// \param name The name of the renderer
    /// \param create A function for creating an instance of that class.
    static void registerRenderer( const std::string &name,
                                  CreateInstanceFunc create ) {
      registerRenderer( HapticsRendererRegistration( name, create ) );
    }

    /// Register a haptics renderer.
    static void registerRenderer( const HapticsRendererRegistration &fr ) {
      registered_renderers->push_back( fr );
    }

    // solution to clean up dummy_context in OpenHapticsRenderer correctly.
    // Could perhaps switch for just calling hdstartscheduler in
    // releaserenderer for OpenHapticsRenderer.
    void cleanUpStuff( HAPIHapticsDevice *hd );

  protected:
    // Creating a new auto_ptr local for this node, because 
    // registered_renderers caused a memory leak and because
    // of the order of setting the static variables the autp_ptr's
    // constructor resets the auto_ptr to 0 eventhough the 
    // registered_renderers has been initilazed, and therefore
    // cause an error making it imposible to use the standard auto_ptr.
    /// \internal
    template<class T>
    class local_auto_ptr{
    private:
      T* ap;    // refers to the actual owned object (if any)
    public:
      typedef T element_type;

      // constructor
      explicit local_auto_ptr (T* ptr = 0) {
        if(!initialized){
          ap=ptr;
        }
      }
      
      // copy constructors (with implicit conversion)
      // - note: nonconstant parameter
      local_auto_ptr (local_auto_ptr& rhs) throw() : ap(rhs.release()) { }

      template<class Y>
      local_auto_ptr (local_auto_ptr<Y>& rhs) throw() : ap(rhs.release()) { }
      
      // assignments (with implicit conversion)
      // - note: nonconstant parameter
      local_auto_ptr& operator= (local_auto_ptr& rhs) throw(){
        if(!initialized){  
          reset(rhs.release());
          return *this;
        }
      }
      template<class Y>
      local_auto_ptr& operator= (local_auto_ptr<Y>& rhs) throw(){
        if(!initialized){
          reset(rhs.release());
          return *this;
        }
      }

      // destructor
      ~local_auto_ptr() throw(){
        delete ap;
      }

      // value access
      T* get() const throw(){
        return ap;
      }
      T& operator*() const throw(){
        return *ap;
      }
      T* operator->() const throw(){
        return ap;
      }

      // release ownership
      T* release() throw(){
        if(!initialized){
          T* tmp(ap);
          ap = 0;
          return tmp;
        }
      }

      // reset value
      void reset (T* ptr=0) throw(){
        if(!initialized){
          if (ap != ptr){
            delete ap;
            ap = ptr;
          }
        }
      }
    };

  protected:
    static local_auto_ptr< std::list< HapticsRendererRegistration > >
    registered_renderers;
    H3DUtil::MutexLock contacts_lock;
    Contacts contacts;
    static bool initialized;

    /// \internal
    class HAPI_API WorkAroundToCleanUpHLContext {
    public:
    virtual ~WorkAroundToCleanUpHLContext() {}
      virtual void cleanUp() {}
    };

    static std::map< HAPIHapticsDevice *,
      std::vector< WorkAroundToCleanUpHLContext * > > clean_up_stuff;
  };
}

#endif
