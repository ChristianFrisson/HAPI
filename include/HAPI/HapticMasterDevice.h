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
/// \file HAPI/HapticMasterDevice.h
/// \brief Header file for HapticMasterDevice.
///
//
//////////////////////////////////////////////////////////////////////////////

#ifndef __HAPI_HAPTICMASTERDEVICE_H__
#define __HAPI_HAPTICMASTERDEVICE_H__

#include <HAPI/HAPIHapticsDevice.h>
#include <H3DUtil/DynamicLibrary.h>

#ifdef HAVE_HAPTIC_MASTER_API

namespace HAPI {

  /// \ingroup HapticsDevices
  /// \class HapticMasterDevice
  /// \brief Interface to the HapticMaster haptics device from Moog/FCS.
  class HAPI_API HapticMasterDevice: public HAPIHapticsDevice {
  public:
    /// Constructor.
    HapticMasterDevice( const std::string &_device_name = "vmd" ):
      device_handle( -1 ),
      device_name( _device_name ),
      com_thread( NULL ),
      com_func_cb_handle( -1 ) {

      // This one is really really stiff.
      max_stiffness = 10000;
    }

    /// Destructor.
    virtual ~HapticMasterDevice() {}

    /// Get the handle to the haptics master.
    inline int getDeviceHandle() {
      return device_handle;
    }

    /// Get the name of this device as specified in servers.db.
    inline std::string getName() {
      return device_name;
    }

    // Access to create spheres on the haptic master. Only temporarily put
    // into this file.
    int createSphere( Vec3 center, double radius,
                      double ext_spring_stiffness,
                      double int_spring_stiffness,
                      double ext_damping_factor,
                      double int_damping_factor,
                      double ext_thickness,
                      double int_thickness );

    int deleteSphere( int sphere );

    int setSphereRadius( int sphere, double radius );
    int setSpherePosition( int sphere, Vec3 pos );

    /// Register this renderer to the haptics renderer database.
    static HapticsDeviceRegistration device_registration;
  
  protected:
    /// Get the device values(position, orientation, etc. )
    // Doxygen documentation reasons for adding HAPI::
    virtual void updateDeviceValues(
      HAPI::HAPIHapticsDevice::DeviceValues &dv, 
      HAPITime dt );

    /// Send forces and torques to render
    virtual void sendOutput( DeviceOutput &dv,
                             HAPITime dt );

    /// Initialize the haptics device. Use the HapticThread class in Threads.h
    /// as the thread for haptic rendering.
    /// \param _thread_frequency is the desired haptic frequency. 
    /// 1000 is the maximum allowed frequency that can be specified. Setting
    /// this parameter to -1 means run as fast as possible. It is recommended
    /// to use the default value for most users.
    /// \returns true if initialization succeeded.
    virtual bool initHapticsDevice( int _thread_frequency = 1000 );

    /// Releases all resources allocated in initHapticsDevice. 
    virtual bool releaseHapticsDevice();

  protected:
    // dll handle to the HapticMasterDriver.dll
    static H3DUtil::DynamicLibrary::LIBHANDLE dll_handle;
    // nr of references to the dll handle.
    static unsigned int dll_references;
    // the device handle for the haptics device.
    int device_handle;
    // the name of this device as specified in servers.db.
    std::string device_name;
    
    /// Callback function for communication thread
    static H3DUtil::PeriodicThread::CallbackCode com_func( void *data );

    /// Callback handle to the com_func callback that is set up
    int com_func_cb_handle;

    /// Thread used to do communication with the haptics device
    H3DUtil::PeriodicThread *com_thread;

    /// Lock for exchanging data with the communication thread.
    H3DUtil::MutexLock com_lock;

    /// Lock in order to not call the hapticmasterdriver at the same 
    /// time from different threads, since the underlying api is 
    /// not thread safe.
    H3DUtil::MutexLock driver_lock;



    /// The current device values updated in the communicataion thread.
    /// Access to this structure must be contained within locking with 
    /// com_lock.
    DeviceValues current_values;

  };
}

#endif

#endif
