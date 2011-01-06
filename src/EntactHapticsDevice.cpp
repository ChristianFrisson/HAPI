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
/// \file EntactHapticsDevice.cpp
/// \brief Cpp file for EntactHapticsDevice.
///
//
//////////////////////////////////////////////////////////////////////////////



#include <HAPI/EntactHapticsDevice.h>

#ifdef HAVE_ENTACTAPI
#include <EntactAPI.h>
#include <sstream>

using namespace HAPI;

namespace EntactHapticsDeviceInternal {
  string libs_array[1] = {"EntactAPI.dll"};
  list< string > entact_libs(libs_array, libs_array + 1 );
}

unsigned int EntactHapticsDevice::nr_device_instances = 0;
int EntactHapticsDevice::nr_entact_devices = 0;
bool EntactHapticsDevice::EAPI_initialized = false;

HAPIHapticsDevice::HapticsDeviceRegistration 
EntactHapticsDevice::device_registration(
                            "Entact",
                            &(newInstance< EntactHapticsDevice >),
                            EntactHapticsDeviceInternal::entact_libs
                            );

EntactHapticsDevice::EntactHapticsDevice( int _serial_number ):
  serial_number( _serial_number ),
  com_thread( NULL ),
  com_func_cb_handle( -1 ),
  device_id( -1 ) {
  // This might have to be changed if they redo it so that their
  // different devices have different maximum stiffness values.
  max_stiffness = 1450;

  nr_device_instances++;
}

/// Destructor.
EntactHapticsDevice::~EntactHapticsDevice() {
  nr_device_instances--;
  if( nr_device_instances == 0 && EAPI_initialized ) {
    freeEAPI();
    EAPI_initialized = false;
  }
}



bool EntactHapticsDevice::initHapticsDevice( int _thread_frequency ) {
  
  if( !EAPI_initialized ) {
    int res = openEAPI( &nr_entact_devices, 25 );
    if( res != EAPI_OK ) {
      stringstream s;
      s << "Warning: Failed to open Entact device. Unable to query for devices." << endl;
      setErrorMsg( s.str() );
      return false;
    }
    EAPI_initialized = true;
  }
 
  if( nr_entact_devices <= 0 ) {
    stringstream s;
    s << "Warning: Failed to open Entact device. No connected devices. ";
    setErrorMsg( s.str() );
    return false;
  }  

  bool device_found = false;

  if( serial_number == -1 ) {
    // Use any of the available devices.
    for( int i = 0; i < nr_entact_devices; i++ ) {
      if ( !isEnabledEAPI( i ) ) {
	device_id = i;
	device_found = true;
	break;
      }
    }
    if( !device_found ) {
      stringstream s;
      s << "Warning: Failed to open Entact device. All devices already in use." << endl;
      setErrorMsg( s.str() );
      return false;
    }
  } else {
    // find device with specified serial number
    for( int i = 0; i < nr_entact_devices; i++ ) {
      // TODO: if( serial_number == devices[i] sn && !isEnabled( devices[i] ) )
      device_id = 0;;
      device_found = true;
      break;
      // TODO: cou;d not find device with serial number
    }
  }

  if( needsCalibration() ) calibrateDevice();
  setModeEAPI( device_id, EAPI_MODE_FC );
  enableDeviceEAPI( device_id );
  
  com_thread = 
    new H3DUtil::PeriodicThread( H3DUtil::ThreadBase::HIGH_PRIORITY, 1000 );
  com_thread->setThreadName( "Entact device com thread" );

  com_func_cb_handle = com_thread->asynchronousCallback( com_func, this );

  return true;
}

bool EntactHapticsDevice::releaseHapticsDevice() {
  HAPIHapticsDevice::disableDevice();
  if( com_thread ) {
    if( com_func_cb_handle != -1 ) {
      com_thread->removeAsynchronousCallback( com_func_cb_handle );
      com_func_cb_handle = -1;
    }
    delete com_thread;
    com_thread = NULL;
  }

  if( device_id != -1 ) {
    disableDeviceEAPI( device_id );
    device_id = -1;
  }
  
  return true;
}

void EntactHapticsDevice::updateDeviceValues( DeviceValues &dv,
                                               HAPITime dt ) {
  HAPIHapticsDevice::updateDeviceValues( dv, dt );

  if( device_id != -1 ) {
    com_lock.lock();
    dv.position = current_values.position;
    dv.orientation = current_values.orientation;
    dv.button_status = current_values.button_status;
    com_lock.unlock();
  }
}

void EntactHapticsDevice::sendOutput( DeviceOutput &dv,
                                   HAPITime dt ) {
  if( device_id != -1 ) {
    com_lock.lock();
    current_values.force = dv.force;
    current_values.torque = dv.torque;
    com_lock.unlock();
  }
}

H3DUtil::PeriodicThread::CallbackCode
EntactHapticsDevice::com_func( void *data ) {
  EntactHapticsDevice *hd = 
    static_cast< EntactHapticsDevice * >( data );
  
  if( hd->device_id != -1 ) {
    double positions[6]; 
    readTaskPositionEAPI( hd->device_id, positions );
    double velocity[6];
    // TODO:
    //readTaskVelocityEAPI( hd->device_id, velocity );

    Rotation orientation = 
      Rotation( 1, 0, 0,  positions[5] ) * 
      Rotation( 0, 1, 0,  positions[4] ) * 
      Rotation( 0, 0, 1, -positions[3] );

    hd->com_lock.lock();

    hd->current_values.position = Vec3( -positions[0], positions[2], positions[1] );
    hd->current_values.velocity = Vec3( -velocity[0], velocity[2], velocity[1] );
    //hd->current_values.button_status = button;
    hd->current_values.orientation = orientation;
 
    Vec3 force = hd->current_values.force;
    Vec3 torque = hd->current_values.torque;
    hd->com_lock.unlock();

    double forces[6] = { -force[0], force[2], force[1], -torque[0], torque[2], torque[1] };
    
    writeForceEAPI( hd->device_id, forces );
  }
  return H3DUtil::PeriodicThread::CALLBACK_CONTINUE;
}

bool EntactHapticsDevice::calibrateDevice() {
  int res = homeDeviceEAPI( device_id );
  return res == EAPI_OK;
}

/// Called to query a device of its calibration status. 
bool EntactHapticsDevice::needsCalibration() {
  int res = isHomedEAPI( device_id );
  return res == EAPI_OK;
}

/// Returns the number of Entact devices that are currently
/// connected to the computer.
int EntactHapticsDevice::getNumberConnectedEntactDevices() {
 if( !EAPI_initialized ) {
   int res = openEAPI( &nr_entact_devices, 25 );
   if( res == EAPI_OK ) {
     EAPI_initialized = true;
   }
  }
  return nr_entact_devices;
}


#endif
