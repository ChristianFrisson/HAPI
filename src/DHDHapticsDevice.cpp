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
/// \file DHDHapticsDevice.cpp
/// \brief Cpp file for DHDHapticsDevice.
///
//
//////////////////////////////////////////////////////////////////////////////

#ifdef HAVE_DHDAPI

#include "DHDHapticsDevice.h"
#include <sstream>
#include <dhdc.h>

#if defined(_MSC_VER) 
#pragma comment( lib, "dhdms.lib" )
#endif

using namespace HAPI;

HAPIHapticsDevice::HapticsDeviceRegistration 
DHDHapticsDevice::device_registration(
                            "DHD",
                            &(newInstance< DHDHapticsDevice >)
                            );

bool DHDHapticsDevice::initHapticsDevice() {
  device_id = dhdOpen();
  if( device_id == -1 ) {
    stringstream s;
    s << "Warning: Failed to open Omega device. Error: " 
      << dhdErrorGetLastStr();
    setErrorMsg( s.str() );
    return false;
  }
  return true;
}

bool DHDHapticsDevice::releaseHapticsDevice() {
  HAPIHapticsDevice::disableDevice();
  if( device_id != -1 ) {
    int id = device_id;
    device_id = -1;
    dhdClose( id );
  }
  return true;
}

void DHDHapticsDevice::updateDeviceValues( DeviceValues &dv,
                                               HAPITime dt ) {
  HAPIHapticsDevice::updateDeviceValues( dv, dt );

  if( device_id != -1 ) {
    double x, y, z;
    dhdGetPosition( &z, &x, &y, device_id );
    
    // convert to millimetres
    dv.position = Vec3( x, y, z ) * 1000;
    // TODO: calculate velocity
    dv.velocity = Vec3( 0, 0, 0 );

    dhdGetOrientationRad( &z, &x, &y, device_id );
    Vec3 r( x, y, z );
    dv.orientation = Rotation( r );

    // TODO: multiple buttons
    dv.button_status = (dhdGetButton( 0, device_id ) == DHD_ON);
  }
}

void DHDHapticsDevice::sendOutput( DeviceOutput &dv,
                                   HAPITime dt ) {
  if( device_id != -1 ) {
    dhdSetForceAndTorque( dv.force.z, dv.force.x, dv.force.y, 
                          dv.torque.z, dv.torque.x, dv.torque.y,
                          device_id );
  }
}


int DHDHapticsDevice::getDeviceType() {
  if( device_id != -1 ) {
    return dhdGetSystemType( device_id );
  }
  return -1;
}

/// Puts the device in RESET mode. In this mode, the user is expected
/// to put the device end-effector at its rest position. This is how
/// the device performs its calibration. 
void DHDHapticsDevice::reset() {
  if( device_id != -1 ) {
    dhdReset( device_id );
  }
}

/// Puts the device in RESET mode and wait for the user to calibrate
/// the device.  Optionally, a timeout can be defined after which the 
/// call returns even if calibration has not occured.
void DHDHapticsDevice::waitForReset( int timeout ) {
  if( device_id != -1 ) {
    dhdWaitForReset( timeout );
  }
}

/// Enable/disable gravity compensation. A value of true enables it.
void DHDHapticsDevice::useGravityCompensation( bool b ) {
  if( device_id != -1 ) {
    dhdSetGravityCompensation( b ? DHD_ON : DHD_OFF,
                               device_id );
  }
}

/// Define the mass of the end-effector. This function is required
/// to provide accurate gravity compensation when custom-made or 
/// modified end-effectors are used.
void DHDHapticsDevice::setEffectorMass( double mass ) {
  if( device_id != -1 ) {
    dhdSetEffectorMass( mass, device_id );
  }
}

/// Enable/disable the device electromagnetic brakes. If enabled
/// the device motor circuits are shortcut to produce electromagnetic
/// viscosity. The viscosity is sufficient to prevent the device from
/// falling too hard onto if forces are disabled abruptly, either by
/// pressing the force button or by action of a safety feature.
void DHDHapticsDevice::useBrakes( bool enable ) {
  if( device_id != -1 ) {
    dhdSetBrakes(  enable ? DHD_ON : DHD_OFF,
                   device_id );
  }
}


#endif
