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
/// \file HaptikHapticsDevice.cpp
/// \brief Cpp file for HaptikHapticsDevice.
///
//
//////////////////////////////////////////////////////////////////////////////


#include <HAPI/HaptikHapticsDevice.h>

#ifdef HAVE_HAPTIK_LIBRARY

using namespace HAPI;

void HaptikHapticsDevice::changeHaptikDevice(
#ifdef HAPTIK_SDK_VERSION
      uint32
#else
      UINT32
#endif
                                            device_id ) {
  if( haptik_device ) {
    RSLib::HaptikDeviceInfo info;
    haptik_device->GetInfo( info );
    int index = -1;

    // find the index of the current device
    for(
#ifdef HAPTIK_SDK_VERSION
      uint32
#else
      UINT32
#endif
         i = 0 ; i<haptik.numberOfDevices ; ++i) {
      if( haptik.device[i].id == info.id ) {
        index = i;
        break;
      }
    }

    assert( index != -1 );

    // if the new device_id is different than the current device
    // disable the old one and initialize the new one.
    if( device_id != info.id ) {
      haptik_device->Release();
      change_haptik_device_lock.lock();
      haptik_device = 
        (RSLib::IHaptikDeviceInterface) 
        haptik.GetDeviceInterface( device_id );
      change_haptik_device_lock.unlock();
    }
  } else {
    change_haptik_device_lock.lock();
    haptik_device = 
      (RSLib::IHaptikDeviceInterface) haptik.GetDeviceInterface( device_id );
    change_haptik_device_lock.unlock();
  }
}

bool HaptikHapticsDevice::initHapticsDevice( int _thread_frequency ) {
  bool return_value = false;
  if( haptik_device ) {
    return_value = haptik_device->Init() == HAPTIK_SUCCESS;
    if( !return_value )
      haptik_device->Release();
  }
  return return_value;
}

bool HaptikHapticsDevice::releaseHapticsDevice() {
  if( haptik_device ) {
    haptik_device->Release();
    return true;
  }
  return false;
}

void HaptikHapticsDevice::updateDeviceValues( DeviceValues &dv,
                                       HAPITime dt ) {
  change_haptik_device_lock.lock();
  HAPIHapticsDevice::updateDeviceValues( dv, dt );
  if( haptik_device ) {
    RSLib::HaptikData data;
    haptik_device->Read( data );
    dv.position = 1e-3 * Vec3( data.position.x, data.position.y, data.position.z );
    dv.velocity = 1e-3 * Vec3( data.velocity.x, data.velocity.y, data.velocity.z );
    dv.button_status = data.buttonMask;
    dv.orientation = Rotation(
      Matrix3( data.matrix.e00, data.matrix.e10, data.matrix.e20,
               data.matrix.e01, data.matrix.e11, data.matrix.e21,
               data.matrix.e02, data.matrix.e12, data.matrix.e22 ) );
  }
  change_haptik_device_lock.unlock();
}

void HaptikHapticsDevice::sendOutput( DeviceOutput &dv,
                               HAPITime dt ) {
  change_haptik_device_lock.lock();
  if( haptik_device ) {
    RSLib::HaptikData data;
    data.forceFeedback.x = (float)dv.force.x;
    data.forceFeedback.y = (float)dv.force.y;
    data.forceFeedback.z = (float)dv.force.z;
    data.torqueFeedback.x = (float)dv.torque.x;
    data.torqueFeedback.y = (float)dv.torque.y;
    data.torqueFeedback.z = (float)dv.torque.z;
    haptik_device->Write( data );
  }
  change_haptik_device_lock.unlock();
}

#endif
