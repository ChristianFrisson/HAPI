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
/// \file HaptionHapticsDevice.h
/// \brief Header file for HaptionHapticsDevice.
///
//
//////////////////////////////////////////////////////////////////////////////

#ifndef __HAPTIONHAPTICSDEVICE_H__
#define __HAPTIONHAPTICSDEVICE_H__

#include <HAPI/HAPIHapticsDevice.h>

#ifdef HAVE_VIRTUOSEAPI

#include <virtuoseAPI.h>

namespace HAPI {

  /// \ingroup HapticsDevices
  /// \class HaptionHapticsDevice
  /// \brief Interface to all haptics devices from Haption, including
  /// the Delta and Omega devices.
  ///
  /// Uses the DHD API to interface to the haptics devices, so every
  /// device supported by DHD API is supported by this class.
  class HAPI_API HaptionHapticsDevice: public HAPIHapticsDevice {
  public:
    /// Constructor.
    HaptionHapticsDevice( const std::string &_ip_address = "192.168.1.1"):
      context( NULL ),
      com_thread( NULL ),
      com_func_cb_handle( -1 ),
      ip_address( _ip_address ) {
        // This might have to be changed if they redo it so that their
        // different devices have different maximum stiffness values.
        max_stiffness = 1450;
      }
    
    /// Destructor.
    virtual ~HaptionHapticsDevice() {}

    //getDeadMan
    //getAlarm
    //isinbounds
    //trackball?

    /// Returns the ip address of the device
    inline std::string getIpAddress() { 
      return ip_address;
    }

    /// Register this renderer to the haptics renderer database.
    static HapticsDeviceRegistration device_registration;
  protected:
    /// Implementation of updateDeviceValues using DHD API to get the values.
    virtual void updateDeviceValues( DeviceValues &dv, HAPITime dt );

    /// Implementation of sendOutput using DHD API to send forces.
    virtual void sendOutput( DeviceOutput &dv,
                             HAPITime dt );

    /// Initialize the haptics device. Use the HapticThread class in Threads.h
    /// as the thread for haptic rendering.
    /// \param _thread_frequency is the desired haptic frequency. 
    /// 1000 is the maximum allowed frequency that can be specified. Setting
    /// this parameter to -1 means run as fast as possible. It is recommended
    /// to use the default value for most users.
    virtual bool initHapticsDevice( int _thread_frequency = 1000 );

    /// Releases all resources allocated in initHapticsDevice. 
    virtual bool releaseHapticsDevice();

    /// The virtouse API context for this device.
    VirtContext context;

    /// The ip address of the virtuose device.
    std::string ip_address;

    /// Callback function for communication thread
    static H3DUtil::PeriodicThread::CallbackCode com_func( void *data );

    /// Callback handle to the com_func callback that is set up
    int com_func_cb_handle;

    /// Thread used to do communication with the haptics device
    H3DUtil::PeriodicThread *com_thread;

    /// Lock for exchanging data with the communication thread.
    H3DUtil::MutexLock com_lock;

    /// The current device values updated in the communicataion thread.
    /// Access to this structure must be contained within locking with 
    /// com_lock.
    DeviceValues current_values;
  };
}

#endif

#endif
