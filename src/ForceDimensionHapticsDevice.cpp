//////////////////////////////////////////////////////////////////////////////
//    Copyright 2004-2014, SenseGraphics AB
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
/// \file ForceDimensionHapticsDevice.cpp
/// \brief Cpp file for ForceDimensionHapticsDevice.
///
//
//////////////////////////////////////////////////////////////////////////////



#include <HAPI/ForceDimensionHapticsDevice.h>
#include <H3DUtil/DynamicLibrary.h>

#ifdef HAVE_DHDAPI
#include <sstream>
#ifdef HAVE_DRDAPI
#include <drdc.h>
#else
#include <dhdc.h>
#endif

using namespace HAPI;

namespace ForceDimensionHapticsDeviceInternal {
#ifdef H3D_WIN64
#ifdef HAVE_DRDAPI
  std::string libs_array[1] = {"drd64.dll"};
#else
  std::string libs_array[1] = {"dhd64.dll"};
#endif
#else
#ifdef HAVE_DRDAPI
  std::string libs_array[1] = {"drd.dll"};
#else
  std::string libs_array[1] = {"dhd.dll"};
#endif
#endif
  std::list< std::string > force_dimension_libs(libs_array, libs_array + 1 );

  Rotation calculateRotationOld( double &rx, double &ry, double &rz ) {
    return Rotation( 1, 0, 0, (float)( H3DUtil::Constants::pi / 4 ) ) *
        Rotation( 0, 0, 1, rz ) *
        Rotation( 1, 0, 0, rx ) *
        Rotation( 0, 1, 0, ry ) *
        Rotation( 1, 0, 0, ( -H3DUtil::Constants::pi / 2 ) );
  }

  Rotation calculateRotationSigma( double &rx, double &ry, double &rz ) {
    return Rotation( 0, 0, 1, rz ) *
           Rotation( 1, 0, 0, rx ) *
           Rotation( 0, 1, 0, ry );
  }

  Vec3 calculateAngularVelocity( double &rx, double &ry, double &rz ) {
    return Rotation( 1, 0, 0, (float)( H3DUtil::Constants::pi / 4 ) ) *
           Rotation( 1, 0, 0, ( -H3DUtil::Constants::pi / 2 ) ) * Vec3( rx, ry, rz );
  }

  Vec3 calculateAngularVelocitySigma( double &rx, double &ry, double &rz ) {
    return Vec3( rx, ry, rz );
  }

  Rotation calculateRotationCustom204( double &rx, double &ry, double &rz ) {
    return Rotation( 0, 0, 1, rz ) *
           Rotation( 1, 0, 0, rx ) *
           Rotation( 0, 1, 0, ry)*
           Rotation( 1, 0, 0, ( -H3DUtil::Constants::pi / 2) );
  }

  Vec3 calculateAngularVelocityCustom204( double &rx, double &ry, double &rz ) {
    return Vec3( rx, ry, rz );
  }
}

HAPIHapticsDevice::HapticsDeviceRegistration 
ForceDimensionHapticsDevice::device_registration(
                            "ForceDimension",
                            &(newInstance< ForceDimensionHapticsDevice >),
                            ForceDimensionHapticsDeviceInternal::force_dimension_libs
                            );

ForceDimensionHapticsDevice::ForceDimensionHapticsDevice():
  device_id( -1 ),
  com_thread( NULL ),
  com_func_cb_handle( -1 ),
  gripper_angle( 0 ),
  gripper_angle_com_thread( 0 ),
  is_autocalibrated( true ), // Default value it true so a user of the class won't trigger autocalibration if the value has not yet been updated;
  is_autocalibrated_com_thread( true ) {
  // This might have to be changed if they redo it so that their
  // different devices have different maximum stiffness values.
  max_stiffness = 1450;
  rotation_func = &ForceDimensionHapticsDeviceInternal::calculateRotationOld;
  angular_velocity_func = &ForceDimensionHapticsDeviceInternal::calculateAngularVelocity;
}

std::vector< int > ForceDimensionHapticsDevice::free_dhd_ids;
int ForceDimensionHapticsDevice::nr_of_connected_dhd_devices = -1;
bool ForceDimensionHapticsDevice::initHapticsDevice( int _thread_frequency ) {
#ifdef WIN32
  /// need to go check if the dll to support this haptic device can be correctly
  /// loaded
  std::list<std::string>::iterator it = device_registration.libs_to_support.begin();
  for( ; it!= device_registration.libs_to_support.end();++it ) {
    if( !H3DUtil::DynamicLibrary::load(*it) ) {
      setErrorMsg("Warning: can not load required DLL for "+ device_registration.name+ "device");
      return false; // if required lib can not be loaed, do not register this device
    }
  }
#endif
  if( nr_of_connected_dhd_devices <= 0 ) {
    nr_of_connected_dhd_devices = dhdGetDeviceCount();
    if( nr_of_connected_dhd_devices <= 0 ) {
      std::stringstream s;
      s << "Warning: Failed to open ForceDimension device. No connected devices. ";
      setErrorMsg( s.str() );
      return false;
    } else {
      for( int i = 0; i < nr_of_connected_dhd_devices; ++i )
        free_dhd_ids.push_back( i );
    }
  } else if( free_dhd_ids.empty() ) {
    std::stringstream s;
    s << "Warning: Failed to open ForceDimension device. All connected devices are "
      << "already initialized.";
    setErrorMsg( s.str() );
    return false;
  }

#ifdef HAVE_DRDAPI
  device_id = drdOpenID( free_dhd_ids.back() );
#else
  device_id = dhdOpenID( free_dhd_ids.back() );
#endif
  if( device_id == -1 ) {
    std::stringstream s;
    s << "Warning: Failed to open ForceDimension device. Error: "
      << dhdErrorGetLastStr();
    setErrorMsg( s.str() );
    return false;
  }
  free_dhd_ids.pop_back();
  
  com_thread = 
    new H3DUtil::PeriodicThread( H3DUtil::ThreadBase::HIGH_PRIORITY, 1000 );
  com_thread->setThreadName( "DHD com thread" );

  com_func_cb_handle = com_thread->asynchronousCallback( com_func, this );

#ifdef DHD_DEVICE_SIGMA331
  int device_type = getDeviceType();
  if( device_type >= DHD_DEVICE_SIGMA331 && device_type <= DHD_DEVICE_SIGMA331 + 5 ) {
    rotation_func = &ForceDimensionHapticsDeviceInternal::calculateRotationSigma;
    angular_velocity_func = &ForceDimensionHapticsDeviceInternal::calculateAngularVelocitySigma;
  } else if( device_type == 204 || device_type == 114 ) { // A customized device that is currently unnamed.
    // Both device 204 and 114 seem to be ok with the same transformation functions.
    rotation_func = &ForceDimensionHapticsDeviceInternal::calculateRotationCustom204;
    angular_velocity_func = &ForceDimensionHapticsDeviceInternal::calculateAngularVelocityCustom204;
  } else {
#endif
    rotation_func = &ForceDimensionHapticsDeviceInternal::calculateRotationSigma;
    angular_velocity_func = &ForceDimensionHapticsDeviceInternal::calculateAngularVelocitySigma;
#ifdef DHD_DEVICE_SIGMA331
  }
#endif

  return true;
}

bool ForceDimensionHapticsDevice::releaseHapticsDevice() {
  HAPIHapticsDevice::disableDevice();
  if( device_id != -1 ) {
    if( com_thread ) {
      if( com_func_cb_handle != -1 ) {
        com_thread->removeAsynchronousCallback( com_func_cb_handle );
        com_func_cb_handle = -1;
      }
      delete com_thread;
      com_thread = NULL;
    }

    int id = device_id;
    device_id = -1;
#ifdef HAVE_DRDAPI
    drdClose( id );
#else
    dhdClose( id );
#endif
    free_dhd_ids.push_back( id );

  }
  return true;
}

void ForceDimensionHapticsDevice::updateDeviceValues( DeviceValues &dv,
                                               HAPITime dt ) {
  HAPIHapticsDevice::updateDeviceValues( dv, dt );

  if( device_id != -1 ) {
    com_lock.lock();
    dv.position = current_values.position;
    dv.velocity = current_values.velocity;
    dv.orientation = current_values.orientation;
    dv.button_status = current_values.button_status;
    gripper_angle = gripper_angle_com_thread;
    is_autocalibrated = is_autocalibrated_com_thread;
    dv.angular_velocity = current_values.angular_velocity;
    com_lock.unlock();
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

// Puts the device in RESET mode. In this mode, the user is expected
// to put the device end-effector at its rest position. This is how
// the device performs its calibration. 
void ForceDimensionHapticsDevice::reset() {
  if( device_id != -1 ) {
    dhdReset( device_id );
  }
}

// Puts the device in RESET mode and wait for the user to calibrate
// the device. Optionally, a timeout can be defined after which the 
// call returns even if calibration has not occured.
void ForceDimensionHapticsDevice::waitForReset( int timeout ) {
  if( device_id != -1 ) {
    dhdWaitForReset( timeout, device_id );
  }
}

// Enable/disable gravity compensation. A value of true enables it.
void ForceDimensionHapticsDevice::useGravityCompensation( bool b ) {
  if( device_id != -1 ) {
    dhdSetGravityCompensation( b ? DHD_ON : DHD_OFF,
                               device_id );
  }
}

// Define the mass of the end-effector. This function is required
// to provide accurate gravity compensation when custom-made or 
// modified end-effectors are used.
void ForceDimensionHapticsDevice::setEffectorMass( double mass ) {
  if( device_id != -1 ) {
    dhdSetEffectorMass( mass, device_id );
  }
}

// Enable/disable the device electromagnetic brakes. If enabled
// the device motor circuits are shortcut to produce electromagnetic
// viscosity. The viscosity is sufficient to prevent the device from
// falling too hard onto if forces are disabled abruptly, either by
// pressing the force button or by action of a safety feature.
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
    double x, y, z, rx, ry, rz, vx, vy, vz, gripper_angle, avx = 0, avy = 0, avz = 0;
    dhdGetPosition( &z, &x, &y, hd->device_id );
    dhdGetOrientationRad( &rz, &rx, &ry, hd->device_id );
    dhdGetLinearVelocity( &vz, &vx, &vy, hd->device_id );
    dhdGetGripperAngleRad( &gripper_angle, hd->device_id );
#ifdef DHD_DEVICE_SIGMA331
    dhdGetAngularVelocityRad( &avz, &avx, &avy, hd->device_id );
#endif
    

#ifdef DHD_DEVICE_SIGMA331
    unsigned int button = ( dhdGetButtonMask( hd->device_id ) & 0xff );
#else
    unsigned int button = 0;
    for( int i = 0; i < H3DMin( DHD_MAX_BUTTONS, 8 ); ++i ) {
      if( dhdGetButton( i, hd->device_id ) == DHD_ON ) {
        button |= 1 << i;
      }
    }
    
#endif
#ifdef HAVE_DRDAPI
    bool _is_autocalibrated = drdIsInitialized( hd->device_id );
#endif
    

    Vec3 position = Vec3( x, y, z );
    Vec3 velocity = Vec3( vx, vy, vz );

    Rotation orientation = hd->rotation_func( rx, ry, rz );
    Vec3 angular_velocity = hd->angular_velocity_func( avx, avy, avz );

    hd->com_lock.lock();

    hd->current_values.position = position;
    hd->current_values.velocity = velocity;
    hd->current_values.button_status = button;
    hd->current_values.orientation = orientation;
    hd->gripper_angle_com_thread = gripper_angle;
    hd->current_values.angular_velocity = angular_velocity;
 
    Vec3 force = hd->current_values.force;
    Vec3 torque = hd->current_values.torque;
#ifdef HAVE_DRDAPI
    hd->is_autocalibrated_com_thread = _is_autocalibrated;
#endif
    hd->com_lock.unlock();

    dhdSetForceAndTorque( force.z, force.x, force.y, 
                          torque.z, torque.x, torque.y,
                          hd->device_id );
  }
  return H3DUtil::PeriodicThread::CALLBACK_CONTINUE;
}

void ForceDimensionHapticsDevice::enableForce( bool enable ) {
  if( device_id != -1 ) {
    dhdEnableForce( enable ? DHD_ON : DHD_OFF,
                    device_id );
  }
}

void ForceDimensionHapticsDevice::setVibration( const HAPIFloat &frequency, const HAPIFloat &amplitude ) {
#ifdef DHD_UNDEFINED // This define does not exist unless vibration support is added, I hope.
  if( device_id != -1 ) {
    dhdSetVibration( frequency, amplitude, 0, device_id );
  }
#endif
}

#endif

bool ForceDimensionHapticsDevice::autoCalibrate() {
#ifdef HAVE_DRDAPI
  if( device_id != -1 ) {
    if( com_thread->removeAsynchronousCallback( com_func_cb_handle ) ) {
      com_func_cb_handle = -1;
      if( drdAutoInit( device_id ) != 0 ) {
        std::stringstream s;
        s << "Warning: Failed to auto calibrate Force Dimension device. Error: "
          << dhdErrorGetLastStr();
        setErrorMsg( s.str() );
        com_func_cb_handle = com_thread->asynchronousCallback( com_func, this );
        return false;
      }
      drdStop();
      com_func_cb_handle = com_thread->asynchronousCallback( com_func, this );
    }
  }
#endif
  return true;
}
