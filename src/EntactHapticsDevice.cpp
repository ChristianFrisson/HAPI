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
/// \file EntactHapticsDevice.cpp
/// \brief Cpp file for EntactHapticsDevice.
///
//
//////////////////////////////////////////////////////////////////////////////



#include <HAPI/EntactHapticsDevice.h>
#include <H3DUtil/DynamicLibrary.h>

#ifdef HAVE_ENTACTAPI
#include <EntactAPI.h>
#include <sstream>

using namespace HAPI;
using namespace std;

// define to do communication with device in separate thread
#define USE_COM_THREAD

namespace EntactHapticsDeviceInternal {
  string libs_array[1] = {"EntactAPI.dll"};
  list< string > entact_libs(libs_array, libs_array + 1 );
}

unsigned int EntactHapticsDevice::nr_device_instances = 0;
int EntactHapticsDevice::nr_entact_devices = 0;
EntactHapticsDevice::EAPIInitializedStyle EntactHapticsDevice::EAPI_initialized = EntactHapticsDevice::ENTACT_UNINITIALIZED;

HAPIHapticsDevice::HapticsDeviceRegistration 
EntactHapticsDevice::device_registration(
                            "Entact",
                            &(newInstance< EntactHapticsDevice >),
                            EntactHapticsDeviceInternal::entact_libs
                            );

EntactHapticsDevice::EntactHapticsDevice( int _serial_number,
                                          string _ip_address ):
  serial_number( _serial_number ),
  ip_address( _ip_address ),
  com_thread( NULL ),
  com_func_cb_handle( -1 ),
  device_id( -1 ) {
  // This might have to be changed if they redo it so that their
  // different devices have different maximum stiffness values.
  max_stiffness = 1450;

  ++nr_device_instances;
}

// Destructor.
EntactHapticsDevice::~EntactHapticsDevice() {
  --nr_device_instances;
  if( nr_device_instances == 0 && EAPI_initialized != ENTACT_UNINITIALIZED ) {
    // Here I am assuming that closeEAPI can be called even if connectDeviceEAPI
    // has been used instead of openEAPI.
    closeEAPI();
    EAPI_initialized = ENTACT_UNINITIALIZED;
  }
}

#define MAX_NR_DEVICES 10
eapi_device_handle handles[MAX_NR_DEVICES]; // detecting up to 10 attached devices


bool EntactHapticsDevice::initHapticsDevice( int _thread_frequency ) {
#ifdef H3D_WINDOWS
  /// need to go check if the dll to support this haptic device can be correctly
  /// loaded
  list<string>::iterator it = device_registration.libs_to_support.begin();
  for( ; it!= device_registration.libs_to_support.end();++it ) {
    if( !H3DUtil::DynamicLibrary::load(*it) ) {
      setErrorMsg("Warning: can not load required DLL for "+ device_registration.name+ "device");
      return false; // if required lib can not be loaed, do not register this device
    }
  }
#endif // H3D_WINDOWS
  if( serial_number != -1 || ip_address == "" ) {
    if( EAPI_initialized == ENTACT_UNINITIALIZED ) {
      nr_entact_devices = openEAPI( handles, MAX_NR_DEVICES );
      if( nr_entact_devices < 0 ) {
        stringstream s;
        s << "Warning: Failed to open Entact device. Unable to query for devices." << endl;
        setErrorMsg( s.str() );
        return false;
      }
      EAPI_initialized = ENTACT_SERIAL_NUMBER;
    }
  } else if( EAPI_initialized == ENTACT_SERIAL_NUMBER ) {
    stringstream s;
    s << "Warning: Failed to open Entact device. " << endl
      << "Tried to initialize with static ip address when previous devices"
      << " were initialized with serial number." << endl;
    setErrorMsg( s.str() );
    return false;
  } else {
      char *_ip_address = new char[ip_address.size()];
      for( unsigned int i = 0; i < ip_address.size(); ++i )
        _ip_address[i] = ip_address[i];
      if (connectDeviceEAPI( handles, nr_entact_devices, _ip_address )==EAPI_ERR) {
        stringstream s;
        s << "Warning: Failed to open Entact device with ip address " << ip_address << ".";
        setErrorMsg( s.str() );
        delete [] _ip_address;
        return false;
      }
      delete [] _ip_address;
      ++nr_entact_devices;
      EAPI_initialized = ENTACT_IP_ADDRESS;
    }
 
  if( nr_entact_devices <= 0 ) {
    stringstream s;
    s << "Warning: Failed to open Entact device. No connected devices. ";
    setErrorMsg( s.str() );
    return false;
  }

  if( EAPI_initialized == ENTACT_SERIAL_NUMBER ) {
    if( serial_number == -1 ) {
      bool device_found = false;
      // Use any of the available devices.
      for( int i = 0; i < nr_entact_devices; ++i ) {
        // This relies on the fact that getModeEAPI returns EAPI_DISABLED_MODE
        // if device is not used, and that define is 0. All other return values are
        // non-zero.
        if( !getModeEAPI( handles[i] ) ) {
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
      if( serial_number < nr_entact_devices ) {
        if( !getModeEAPI( handles[serial_number] ) ) {
          device_id = serial_number;
        } else {
          stringstream s;
          s << "Warning: Failed to open Entact device with serial number " << serial_number
            << "Device is already in use or an error occured when polling its mode." << endl;
          setErrorMsg( s.str() );
          return false;
        }
      } else {
        stringstream s;
        s << "Warning: Failed to open Entact device with serial number " << serial_number
          << "The serial number is greater than or equal "
          << "to the number of connected devices (" << nr_entact_devices << ")." << endl;
        setErrorMsg( s.str() );
        return false;
      }
    }
  }

  if( needsCalibration() ) {
    if( !calibrateDevice() ) {
      stringstream s;
      s << "Warning:  Failed to open Entact device. Homing failed. ";
      setErrorMsg( s.str() );
      return false;
    }
  }

  if( setModeEAPI( handles[device_id], EAPI_FORCECONTROL_MODE ) == EAPI_ERR ) {
    stringstream s;
    s << "Warning:  Failed to open Entact device.  Failed to change to force control mode.";
    setErrorMsg( s.str() );
    return false;
  }
        
#ifdef USE_COM_THREAD
  com_thread = 
    new H3DUtil::PeriodicThread( H3DUtil::ThreadBase::HIGH_PRIORITY, 1000 );
  com_thread->setThreadName( "Entact device com thread" );

  com_func_cb_handle = com_thread->asynchronousCallback( com_func, this );
#endif
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
    setModeEAPI( handles[device_id], EAPI_DISABLED_MODE );
    device_id = -1;
  }
  
  return true;
}

void EntactHapticsDevice::updateDeviceValues( DeviceValues &dv,
                                               HAPITime dt ) {
  HAPIHapticsDevice::updateDeviceValues( dv, dt );

  if( device_id != -1 ) {
#ifdef USE_COM_THREAD
    com_lock.lock();
#else
    com_func( this );
#endif
   
    dv.position = current_values.position;
    dv.orientation = current_values.orientation;
    dv.button_status = current_values.button_status;
    dv.velocity = current_values.velocity;
#ifdef USE_COM_THREAD
    com_lock.unlock();
#endif
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
    double positions[12];
    // What to do if these functions fail? check against EAPI_ERR
    readTaskPositionEAPI( handles[hd->device_id], positions, 12 );

    double velocity[6];

    readTaskVelocityEAPI( handles[hd->device_id], velocity, 6 );

    Matrix3 orientationMatrix ( positions[3], positions[4], positions[5],
        positions[6], positions[7], positions[8],
        positions[9], positions[10], positions[11] );
    
    Rotation orientation( orientationMatrix );

    
    orientation = Rotation(1,0,0,-H3DUtil::Constants::pi/2) * orientation;
    orientation = Rotation(0,1,0, -H3DUtil::Constants::pi/2) * orientation;

#ifdef USE_COM_THREAD

    hd->com_lock.lock();
#endif

    hd->current_values.position = Vec3( positions[1], positions[2], positions[0] - 210 ) * 1e-3; // m -> mm + origin_offset
    hd->current_values.velocity = Vec3( velocity[1], velocity[2], velocity[0] ) * 1e-3; // mm/s -> m/s
    //hd->current_values.button_status = button;
    hd->current_values.orientation = orientation;
    Vec3 force = hd->current_values.force;    // N -> mN
    Vec3 torque = hd->current_values.torque * 1000;  // Nm -> Nmm

#ifdef USE_COM_THREAD
    hd->com_lock.unlock();
#endif

    double forces[6] = { force[2], force[0], force[1], torque[2], torque[0], torque[1] };
    writeForceEAPI( handles[hd->device_id], forces, 6 );
  }

  return H3DUtil::PeriodicThread::CALLBACK_CONTINUE;
}

bool EntactHapticsDevice::calibrateDevice() {
  int i = 0;
  int res = homeDeviceEAPI( handles[device_id] );

  // home device calls fails sometimes even though it should not
  // so try a number of times before deciding that it could not home
  while( res == EAPI_ERR && i < 20 ) {
    res = homeDeviceEAPI( handles[device_id] );
    ++i;
  }

  return i < 20;
}

/// Called to query a device of its calibration status. 
bool EntactHapticsDevice::needsCalibration() {
  int res = isHomedEAPI( handles[device_id] );
  return res != 1;
}

/// Returns the number of Entact devices that are currently
/// connected to the computer.
int EntactHapticsDevice::getNumberConnectedEntactDevices() {
  if( EAPI_initialized == ENTACT_UNINITIALIZED ) {
    nr_entact_devices = openEAPI( handles, MAX_NR_DEVICES );
    if( nr_entact_devices >= 0 ) {
      EAPI_initialized = ENTACT_SERIAL_NUMBER;
    }
  }
  return nr_entact_devices;
}


#endif
