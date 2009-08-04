//////////////////////////////////////////////////////////////////////////////
//    Copyright 2004-2007, SenseGraphics AB
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
/// \file FalconHapticsDevice.h
/// \brief Header file for FalconHapticsDevice.
///
//
//////////////////////////////////////////////////////////////////////////////

#ifndef __FALCONHAPTICSDEVICE_H__
#define __FALCONHAPTICSDEVICE_H__

#include <HAPI/HAPIHapticsDevice.h>

#ifdef HAVE_FALCONAPI

#include <hdl/hdl.h>

namespace HAPI {

  /// \ingroup HapticsDevices
  /// \class FalconHapticsDevice
  /// \brief Interface the Falcon haptics device from Novint.
  ///
  /// Use Novints HDAL SDK to interface with a Falcon.
  class HAPI_API FalconHapticsDevice: public HAPIHapticsDevice {
  public:
    /// \ingroup Others
    /// \class FalconThread
    /// \brief A singleton class providing an interface to the
    /// Falcon servo loop and thread.
    ///
    /// It is used by the FalconHapticsDevice and
    /// uses its own thread handling. Since only one instance of the Falcon
    /// servo loop exists it is a singleton class.
    class HAPI_API FalconThread : public H3DUtil::HapticThreadBase,
      public H3DUtil::PeriodicThreadBase {

    private:
      FalconThread():
           is_active( false ) {
             // the Falcon thread should not be added unless the Falcon servo
             // loop has started.
             sg_lock.lock();
             threads.pop_back();
             sg_lock.unlock();
           }
    public:

      /// Get the singleton instance of FalconThread.
      static FalconThread *getInstance() {
        return singleton.get();
      }

      /// If the servo loop has been started true is returned.
      inline bool isActive() { return is_active; }

      /// Set the flag indicating if the Falcon servo loop has been
      /// started or not.
      /// \param _active Value used to set the flag.
      void setActive( bool _active );

      /// Add a callback function to be executed in this thread. The calling
      /// thread will wait until the callback function has returned before 
      /// continuing. 
      virtual void synchronousCallback( CallbackFunc func, void *data );

      /// Add a callback function to be executed in this thread. The calling
      /// thread will continue executing after adding the callback and will 
      /// not wait for the callback function to execute.
      /// Returns a handle to the callback that can be used to remove
      /// the callback.
      virtual int asynchronousCallback( CallbackFunc func, void *data );

      /// Attempts to remove a callback. returns true if succeded. returns
      /// false if the callback does not exist. This function should be handled
      /// with care. It can remove the wrong callback if the callback that
      /// returned the callback_handle id is removed and a new callback is
      /// added. Callbacks are removed if they return CALLBACK_DONE or a call
      /// to this function is made.
      virtual bool removeAsynchronousCallback( int callback_handle );
    protected:
      static H3DUtil::PeriodicThread::CallbackCode setThreadId( void * _data );
      static auto_ptr< FalconThread > singleton;
      bool is_active;
    };

    /// Constructor.
    /// \param _device_name The name of the device as specified in the hdal.ini file.
    /// A _device_name of "" will use the first available device.
    FalconHapticsDevice( string _device_name ):
      device_name( _device_name ),
      device_index( -1 ) {
    }

    /// Constructor.
    /// \param index The index of the falcon devices connected to use. The devices 
    /// are ordered alphabetically on serial number of the device.
    FalconHapticsDevice( int index = 0 ):
      device_index( index ) {
    }

    /// Destructor.
    virtual ~FalconHapticsDevice() {}

    /// Returns the HDL device handle for this device.
    inline HDLDeviceHandle getDeviceHandle() { 
      return device_handle;
    }

    /// Returns the name of the device.
    virtual const string &getDeviceName() {
      return device_name;
    }

    /// Register this renderer to the haptics renderer database.
    static HapticsDeviceRegistration device_registration;

    /// Get the device model of the device. Undefined if device not
    /// initialized.
    inline string getDeviceModel() { return device_model; }

    /// Get the workspace dimensions of the device, i.e. the
    /// mechanical limits of the device. Undefined if
    /// device not initialized.
    /// \param min The minimum values of the workspace dimensions.
    /// \param max The maximum values of the workspace dimensions.
    inline void getWorkspaceDimensions( Vec3 &min, Vec3&max ) {
      min = workspace_min;
      max = workspace_max;
    }

  protected:
    string device_model;
    Vec3 workspace_max;
    Vec3 workspace_min;
    
    /// Implementation of updateDeviceValues using HDAL API to get the values.
    virtual void updateDeviceValues( DeviceValues &dv, HAPITime dt );

    /// Implementation of sendOutput using HDAL API to send forces.
    virtual void sendOutput( DeviceOutput &dv,
                             HAPITime dt );

    /// Implementation of initHapticsDevice using HDAL API.
    /// \param _thread_frequency is the desired haptic frequency. 
    /// Note that thread_frequency can not be set in HDAL API when this is
    /// implemented so the _thread_frequency argument will not be used.
    virtual bool initHapticsDevice( int _thread_frequency = 1000 );

    /// Releases all resources allocated in initHapticsDevice. 
    virtual bool releaseHapticsDevice();

    /// The device name for this device.
    string device_name;
    
    /// The device index to use for this device. 
    int device_index;
    
    /// The device handle for this device.
    HDLDeviceHandle device_handle;

    /// Flag to know if the scheduler is started.
    static bool hdl_started;

    /// Counts the number of initialized falcon devices.
    /// Used to know when to stop scheduler.
    static int nr_of_initalized;
  };
}

#endif

#endif
