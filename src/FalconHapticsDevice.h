//////////////////////////////////////////////////////////////////////////////
//    Copyright 2004-2007, SenseGraphics AB
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
/// \file FalconHapticsDevice.h
/// \brief Header file for FalconHapticsDevice.
///
//
//////////////////////////////////////////////////////////////////////////////

#ifndef __FALCONHAPTICSDEVICE_H__
#define __FALCONHAPTICSDEVICE_H__

#include <HAPIHapticsDevice.h>

#ifdef HAVE_FALCONAPI

#include <hdl/hdl.h>

namespace HAPI {

  /// \class FalconHapticsDevice
  /// \brief Interface the Falcon haptics device from Novint.
  class HAPI_API FalconHapticsDevice: public HAPIHapticsDevice {
  public:
  /// FalconThread is a singleton class providing an interface to the scheduler
  /// and thread. It is used by the FalconHapticsDevice and uses its own thread
  /// handling. Since only one instance of the Falcon scheduler exists it is a
  /// singleton class.
  class HAPI_API FalconThread : public H3DUtil::HapticThreadBase,
                                public H3DUtil::PeriodicThreadBase {

  private:
    FalconThread():
      is_active( false ) {
      // the hl thread should not be added unless the hd scheduler
      // has started.
      sg_lock.lock();
      threads.pop_back();
      sg_lock.unlock();
    }
  public:
    /// Get the singleton instance of FalconThread.
    static FalconThread *getInstance() {
      return singleton;
    }

    /// If the scheduler has been started true is returned.
    inline bool isActive() { return is_active; }

    /// Set the flag indicating if the hd scheduler has been
    /// started or not.
    void setActive( bool _active );

    /// Add a callback function to be executed in this thread. The calling
    /// thread will wait until the callback function has returned before 
    /// continuing. 
    virtual void synchronousCallback( CallbackFunc func, void *data );

    /// Add a callback function to be executed in this thread. The calling
    /// thread will continue executing after adding the callback and will 
    /// not wait for the callback function to execute. 
    virtual void asynchronousCallback( CallbackFunc func, void *data );
  protected:
    static H3DUtil::PeriodicThread::CallbackCode setThreadId( void * _data );
    static FalconThread *singleton;
    bool is_active;
  };

    /// Constructor.
    /// device_name is the name of the device, as defined in the 
    /// "Falcon Configuration" tool. A device_name of "" will use the first available
    /// device.
    FalconHapticsDevice( string _device_name = "" ):
      device_name( _device_name ) {
    }

    /// Destructor.
    virtual ~FalconHapticsDevice() {}

    /// Returns the HD device handle for this device.
    inline HDLDeviceHandle getDeviceHandle() { 
      return device_handle;
    }

    /// Returns the name of the device.
    virtual const string &getDeviceName() {
      return device_name;
    }

    /// Register this renderer to the haptics renderer database.
    //static HapticsDeviceRegistration device_registration;

    /// Get the device model of the device. Undefined if device not
    /// initialized.
    inline string getDeviceModel() { return device_model; }

    /// Get the workspace dimensions of the device, i.e. the
    /// mechanical limits of the device. Undefined if
    /// device not initialized.
    inline void getWorkspaceDimensions( Vec3 &min, Vec3&max ) {
      min = workspace_min;
      max = workspace_max;
    }

  protected:
    string device_model;
    Vec3 workspace_max;
    Vec3 workspace_min;
    
    /// Implementation of updateDeviceValues using HD API to get the values.
    virtual void updateDeviceValues( DeviceValues &dv, HAPITime dt );

    /// Implementation of sendOutput using HD API to send forces.
    virtual void sendOutput( DeviceOutput &dv,
                             HAPITime dt );

    /// Implementation of initHapticsDevice using HD API.
    virtual bool initHapticsDevice();

    /// Releases all resources allocated in initHapticsDevice. 
    virtual bool releaseHapticsDevice();

    /// The device name for this device.
    string device_name;
    
    /// The device handle for this device.
     HDLDeviceHandle device_handle;

    /// Handle for the callback for rendering ForceEffects.  
    //    vector< HDCallbackCode > hd_handles; 
  };
}

#endif

#endif
