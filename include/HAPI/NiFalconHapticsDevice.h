//////////////////////////////////////////////////////////////////////////////
//    Copyright 2009-2019, SenseGraphics AB
//    Copyright 2008-2009, Kyle Machulis
//    Copyright 2009, Karljohan Lundin Palmerius
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
/// \file NiFalconHapticsDevice.h
/// \brief Header file for NiFalconHapticsDevice.
///
//
//////////////////////////////////////////////////////////////////////////////

#ifndef __NIFALCONHAPTICSDEVICE_H__
#define __NIFALCONHAPTICSDEVICE_H__

#include <HAPI/HAPIHapticsDevice.h>

#ifdef HAVE_NIFALCONAPI

// forward declaration
namespace libnifalcon {
  class FalconDevice;
}

namespace HAPI {
 
  /// \ingroup HapticsDevices
  /// \class NiFalconHapticsDevice
  /// \brief Interface to the Falcon device through the open source library
  /// libAnyFalcon. Works on linux.
  class HAPI_API NiFalconHapticsDevice: public HAPIHapticsDevice {
  public:

    /// Returns the current number of connected falcon devices.
    static unsigned int getNrConnectedFalconDevices();
    
    /// Constructor. device_index is the index of falcon device
    /// connected. Should not be larger than getNrConnectedFalconDevices() - 1.
    NiFalconHapticsDevice( unsigned int device_index = 0 );

    /// Destructor.
    virtual ~NiFalconHapticsDevice();
    /// Returns the index of the falcon device the instance of this class
    /// refers to.
    inline unsigned int getDeviceIndex() {
      return index;
    }

    /// Set the index of the falcon device the instance of this class
    /// refers to. This call is only valid before the device is initialized.
    /// If it is called after initialization it will do nothing.
    /// It returns true on success, and false otherwise.
    bool setDeviceIndex( unsigned int index );

    /// Register this renderer to the haptics renderer database.
    static HapticsDeviceRegistration device_registration;

  protected:
    /// Implementation of updateDeviceValues using DHD API to get the values.
    virtual void updateDeviceValues( HAPIHapticsDevice::DeviceValues &dv,
                                     HAPITime dt );

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

    /// The libnifalcon device class. Cannot use auto_ptr here because
    /// the class is forward declared.
    libnifalcon::FalconDevice * device;

    /// The index of the Falcon device that the instance of the class
    /// refers to.
    unsigned int index;
  };
}

#endif //HAVE_NIFALCONAPI
#endif
