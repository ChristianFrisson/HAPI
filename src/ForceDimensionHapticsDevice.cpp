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
/// \file ForceDimensionHapticsDevice.cpp
/// \brief Cpp file for ForceDimensionHapticsDevice.
///
//
//////////////////////////////////////////////////////////////////////////////



#include <HAPI/ForceDimensionHapticsDevice.h>

#ifdef HAVE_DHDAPI
#include <sstream>
#include <dhdc.h>

#if defined(_MSC_VER) 
#pragma comment( lib, "dhdms.lib" )
#endif

using namespace HAPI;

namespace ForceDimensionHapticsDeviceInternal {
  string libs_array[1] = {"dhd.dll"};
  list< string > force_dimension_libs(libs_array, libs_array + 1 );
}

HAPIHapticsDevice::HapticsDeviceRegistration 
ForceDimensionHapticsDevice::device_registration(
                            "ForceDimension",
                            &(newInstance< ForceDimensionHapticsDevice >),
                    ForceDimensionHapticsDeviceInternal::force_dimension_libs
                            );

bool ForceDimensionHapticsDevice::initHapticsDevice( int _thread_frequency ) {
  device_id = dhdOpen();
  if( device_id == -1 ) {
    stringstream s;
    s << "Warning: Failed to open Omega device. Error: " 
      << dhdErrorGetLastStr();
    setErrorMsg( s.str() );
    return false;
  }

#ifdef WIN32
  com_thread = 
    new H3DUtil::PeriodicThread(  THREAD_PRIORITY_ABOVE_NORMAL, 1000 );
#else
  com_thread = new H3DUtil::PeriodicThread( 20, 1000 );
#endif

  com_func_cb_handle = com_thread->asynchronousCallback( com_func, this );

  return true;
}

bool ForceDimensionHapticsDevice::releaseHapticsDevice() {
  HAPIHapticsDevice::disableDevice();
  if( device_id != -1 ) {
    if( com_thread ) {
      if( com_func_cb_handle != -1 )
        com_thread->removeAsynchronousCallback( com_func_cb_handle );
      delete com_thread;
      com_thread = NULL;
    }

    int id = device_id;
    device_id = -1;
    dhdClose( id );
    
  }
  return true;
}

void ForceDimensionHapticsDevice::updateDeviceValues( DeviceValues &dv,
                                               HAPITime dt ) {
  HAPIHapticsDevice::updateDeviceValues( dv, dt );

  if( device_id != -1 ) {
    com_lock.lock();
    dv.position = current_values.position;
    dv.orientation = current_values.orientation;
    dv.button_status = current_values.button_status;
    com_lock.unlock();
    calculateVelocity( dv, dt );
  }
}

void ForceDimensionHapticsDevice::sendOutput( DeviceOutput &dv,
                                   HAPITime dt ) {
  if( device_id != -1 ) {
    com_lock.lock();
    current_values.force = dv.force;
    current_values.torque = dv.torque;
    com_lock.unlock();
  }
}


int ForceDimensionHapticsDevice::getDeviceType() {
  if( device_id != -1 ) {
    return dhdGetSystemType( device_id );
  }
  return -1;
}

/// Puts the device in RESET mode. In this mode, the user is expected
/// to put the device end-effector at its rest position. This is how
/// the device performs its calibration. 
void ForceDimensionHapticsDevice::reset() {
  if( device_id != -1 ) {
    dhdReset( device_id );
  }
}

/// Puts the device in RESET mode and wait for the user to calibrate
/// the device.  Optionally, a timeout can be defined after which the 
/// call returns even if calibration has not occured.
void ForceDimensionHapticsDevice::waitForReset( int timeout ) {
  if( device_id != -1 ) {
    dhdWaitForReset( timeout );
  }
}

/// Enable/disable gravity compensation. A value of true enables it.
void ForceDimensionHapticsDevice::useGravityCompensation( bool b ) {
  if( device_id != -1 ) {
    dhdSetGravityCompensation( b ? DHD_ON : DHD_OFF,
                               device_id );
  }
}

/// Define the mass of the end-effector. This function is required
/// to provide accurate gravity compensation when custom-made or 
/// modified end-effectors are used.
void ForceDimensionHapticsDevice::setEffectorMass( double mass ) {
  if( device_id != -1 ) {
    dhdSetEffectorMass( mass, device_id );
  }
}

/// Enable/disable the device electromagnetic brakes. If enabled
/// the device motor circuits are shortcut to produce electromagnetic
/// viscosity. The viscosity is sufficient to prevent the device from
/// falling too hard onto if forces are disabled abruptly, either by
/// pressing the force button or by action of a safety feature.
void ForceDimensionHapticsDevice::useBrakes( bool enable ) {
  if( device_id != -1 ) {
    dhdSetBrakes(  enable ? DHD_ON : DHD_OFF,
                   device_id );
  }
}

H3DUtil::PeriodicThread::CallbackCode
ForceDimensionHapticsDevice::com_func( void *data ) {
  ForceDimensionHapticsDevice *hd = 
    static_cast< ForceDimensionHapticsDevice * >( data );
  
  if( hd->device_id != -1 ) {
    double x, y, z, rx, ry, rz;
    dhdGetPosition( &z, &x, &y, hd->device_id );
    dhdGetOrientationRad( &rz, &rx, &ry, hd->device_id );    
    // TODO: multiple buttons
    bool button = (dhdGetButton( 0, hd->device_id ) == DHD_ON);
    
    Rotation orientation = Rotation( Vec3( rx, ry, rz ) );
    // convert to millimetres
    Vec3 position = Vec3( x, y, z ) * 1000;

    hd->com_lock.lock();

    hd->current_values.position = position;
    hd->current_values.velocity = Vec3( 0, 0, 0 );
    hd->current_values.button_status = button;
 
    Vec3 force = hd->current_values.force;
    Vec3 torque = hd->current_values.torque;
    hd->com_lock.unlock();

    dhdSetForceAndTorque( force.z, force.x, force.y, 
                          torque.z, torque.x, torque.y,
                          hd->device_id );
                          }
  return H3DUtil::PeriodicThread::CALLBACK_CONTINUE;
}


#endif
