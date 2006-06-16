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
/// \file HaptikDevice.cpp
/// \brief Cpp file for HaptikDevice.
///
//
//////////////////////////////////////////////////////////////////////////////


#include <HaptikDevice.h>

#ifdef HAVE_HAPTIK_LIBRARY

using namespace H3D;

void HaptikDevice::changeHaptikDevice( UINT32 device_id ) {
  if( haptik_device ) {
    RSLib::HaptikDeviceInfo info;
    haptik_device->GetInfo( info );
    int index = -1;

    // find the index of the current device
    for(UINT32 i = 0 ; i<haptik.numberOfDevices ; i++) {
      if( haptik.device[i].id == info.id ) {
        index = i;
        break;
      }
    }

    assert( index != -1 );

    // if the new device_id is different than the current device
    // disable the old one and initialize the new one.
    if( device_id != index && device_id != info.id ) {
      haptik_device->Release();
      haptik_device = 
        (RSLib::IHaptikDeviceInterface) 
        haptik.GetDeviceInterface( device_id );
    }
  } else {
    haptik_device = 
      (RSLib::IHaptikDeviceInterface) haptik.GetDeviceInterface( device_id );
  }
}

void HaptikDevice::initDevice() {
  if( haptik_device ) 
    haptik_device->Init();
}

void HaptikDevice::enableDevice() {
  if( haptik_device )
    haptik_device->Start();
}

void HaptikDevice::disableDevice() {
  if( haptik_device )
    haptik_device->Stop();
}

/// Perform cleanup and let go of all device resources that are allocated.
/// After a call to this function no haptic rendering can be performed on
/// the device until the initDevice() function has been called again.
void HaptikDevice::releaseDevice() {
  if( haptik_device )
    haptik_device->Release();
}

Vec3d HaptikDevice::getPosition() {
    if( haptik_device ) {
    RSLib::HaptikData data;
    haptik_device->Read( data );
    return Vec3d( data.position.x, data.position.y, data.position.z );
  } else {
    return Vec3d( 0, 0, 0 );
  }
}

/// Get the velocity of the haptics device. Only to be called in the 
/// haptics loop.
Vec3d HaptikDevice::getVelocity() {
  if( haptik_device ) {
    RSLib::HaptikData data;
    haptik_device->Read( data );
    return Vec3d( data.velocity.x, data.velocity.y, data.velocity.z );
  } else {
    return Vec3d( 0, 0, 0 );
  }
}

H3DInt32 HaptikDevice::getButtonStatus() {
  if( haptik_device ) {
    RSLib::HaptikData data;
    haptik_device->Read( data );
    return data.buttonMask;
  } else {
    return 0;
  }
}

/// Get the orientation of the haptics device. Only to be called in the 
/// haptics loop.
Rotation HaptikDevice::getOrientation() {
    if( haptik_device ) {
    RSLib::HaptikData data;
    haptik_device->Read( data );
    RSLib::Math::Vector4 &r0 = data.matrix.Row( 0 );
    RSLib::Math::Vector4 &r1 = data.matrix.Row( 1 );
    RSLib::Math::Vector4 &r2 = data.matrix.Row( 2 );
    Matrix3f m( r0.x, r1.x, r2.x,
                r0.y, r1.y, r2.y,
                r0.z, r1.z, r2.z );
    return Rotation( m );
  } else {
    return Rotation( 1, 0, 0, 0 );
  }
}


/// Send the force to render on the haptics device. Only to be called in the 
/// haptics loop.
void HaptikDevice::sendForceToDevice( const Vec3d &f ) {
  if( haptik_device ) {
    RSLib::HaptikData data;
    data.forceFeedback.x = f.x;
    data.forceFeedback.y = f.y;
    data.forceFeedback.z = f.z;
    data.torqueFeedback.x = current_torque.x;
    data.torqueFeedback.y = current_torque.y;
    data.torqueFeedback.z = current_torque.z;
    haptik_device->Write( data );
  }
}

/// Send the torque to render on the haptics device. Only to be called in the 
/// haptics loop.
void HaptikDevice::sendTorqueToDevice( const Vec3d &t ) {
  if( haptik_device ) {
    RSLib::HaptikData data;
    haptik_device->Read( data );
    data.forceFeedback.x = current_force.x;
    data.forceFeedback.y = current_force.y;
    data.forceFeedback.z = current_force.z;
    data.torqueFeedback.x = t.x;
    data.torqueFeedback.y = t.y;
    data.torqueFeedback.z = t.z;
    haptik_device->Write( data );
  }
}

#endif
