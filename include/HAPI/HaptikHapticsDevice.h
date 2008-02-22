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
/// \file HaptikHapticsDevice.h
/// \brief Header file for HaptikHapticsDevice.
///
//
//////////////////////////////////////////////////////////////////////////////

#ifndef __HAPI_HAPTIKHAPTICSDEVICE_H__
#define __HAPI_HAPTIKHAPTICSDEVICE_H__

#include <HAPI/HAPIHapticsDevice.h>

#ifdef HAVE_HAPTIK_LIBRARY
#include <RSLib/Haptik.hpp>

namespace HAPI {

  /// \class HaptikHapticsDevice
  /// HaptikHapticsDevice uses the haptik library (www.haptiklibrary.org)
  /// to interface towards devices.
  class HAPI_API HaptikHapticsDevice: public HAPIHapticsDevice {
  public:

    /// Constructor.
    HaptikHapticsDevice(): 
      haptik_device( NULL ) {
      changeHaptikDevice( HAPTIK_DEFAULT_DEVICE );
    }

    /// Destructor. Stops haptics rendering and remove callback functions.
    virtual ~HaptikHapticsDevice() {
      disableDevice();
      releaseDevice();
    }

    /// Used in order to change what input device is used.
    void changeHaptikDevice( uint32 device_id );

    /// Get the index of the device currently used. The index is an index
    /// in the device array in an Haptik object.
    inline int getHaptikIndex() {
      int dev_index = -1;
      if( haptik_device ) {
        RSLib::HaptikDeviceInfo info;
        haptik_device->GetInfo( info );
        for(uint32 i = 0 ; i< haptik.numberOfDevices ; i++) {
          if( haptik.device[i].id == info.id ) {
            dev_index = i;
            break;
          }
        }
      }
      return dev_index;
    }

  protected:

    /// Initialize the haptics device.
    /// \param _thread_frequency is the desired haptic frequency.
    virtual bool initHapticsDevice( int _thread_frequency = 1000 );

    /// Release all resources allocated to the haptics device.
    virtual bool releaseHapticsDevice();

    /// Implementation of updateDeviceValues using Haptik to get the values.
    virtual void updateDeviceValues( DeviceValues &dv,
                                     HAPITime dt );

    /// Implementation of sendOutput using Haptik to send forces.
    virtual void sendOutput( DeviceOutput &dv,
                             HAPITime dt );

    RSLib::Haptik haptik;

    /// The device currently in use.
    RSLib::IHaptikDeviceInterface haptik_device;

    /// Lock when changing device.
    H3DUtil::MutexLock change_haptik_device_lock;
  };
}

#endif
#endif
