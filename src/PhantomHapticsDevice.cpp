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
/// \file PhantomHapticsDevice.cpp
/// \brief Cpp file for PhantomHapticsDevice.
///
//
//////////////////////////////////////////////////////////////////////////////


#include <HAPI/PhantomHapticsDevice.h>
#include <H3DUtil/DynamicLibrary.h>

#ifdef HAVE_OPENHAPTICS
#include <HAPI/HLThread.h>
#include <sstream>

using namespace HAPI;

namespace PhantomDeviceInternal {
  std::string libs_array[1] = {"HD.dll"};
  std::list< std::string > phantom_device_libs(libs_array, libs_array + 1 );

  // Callback function that starts a new hd frame. It is used in order to 
  // encapsulate all HD API callback function within a hdBeginFrame/hdEndFrame
  // pair in order to only get one frame per loop.
  HDCallbackCode HDCALLBACK beginFrameCallback( void *data ) {
    PhantomHapticsDevice *hd = static_cast< PhantomHapticsDevice * >( data );
    hdBeginFrame( hd->getDeviceHandle() );
    return HD_CALLBACK_CONTINUE;
  }
}

// Callback function that ends a hd frame. It is used in order to 
// encapsulate all HD API callback function within a hdBeginFrame/hdEndFrame
// pair in order to only get one frame per loop.
HDCallbackCode HDCALLBACK PhantomHapticsDevice::endFrameCallback( void *data ){
  // Makes the call to hapticRenderingCallback here in order to be sure
  // that openhaptics own rendering has already been done so the correct
  // force is used when using OpenHapticsRenderer.
  PhantomHapticsDevice *hd = static_cast< PhantomHapticsDevice * >( data );
  if( hd->inCalibrationMode() && hd->needsCalibration() ) {
    hdEndFrame( hd->getDeviceHandle() );
    hd->calibrateDeviceInternal();
  } else {
    HAPIHapticsDevice::hapticRenderingCallback(
      hd->haptic_rendering_callback_data );
    hdEndFrame( hd->getDeviceHandle() );
  }
  return HD_CALLBACK_CONTINUE;
}

HAPIHapticsDevice::HapticsDeviceRegistration 
PhantomHapticsDevice::device_registration(
                            "Phantom",
                            &(newInstance< PhantomHapticsDevice >),
                            PhantomDeviceInternal::phantom_device_libs
                            );

bool PhantomHapticsDevice::scheduler_started = false;
bool PhantomHapticsDevice::enable_start_scheduler = true;
int PhantomHapticsDevice::nr_of_scheduled = 0;

bool PhantomHapticsDevice::initHapticsDevice( int _thread_frequency ) {
  #ifdef H3D_WINDOWS
    /// need to go check if the dll to support this haptic device can be correctly
    /// loaded
    std::list<std::string>::iterator it = device_registration.libs_to_support.begin();
    for( ; it!= device_registration.libs_to_support.end();++it ) {
      if( !H3DUtil::DynamicLibrary::load(*it) ) {
        setErrorMsg("Warning: can not load required DLL for "+ device_registration.name+ "device");
        return false; // if required lib can not be loaed, do not register this device
      }
    }
  #endif // H3D_WINDOWS
  hdapi_version = hdGetString( HD_VERSION );
  device_handle = hdInitDevice( device_name == "" ? 
                                HD_DEFAULT_DEVICE : device_name.c_str() );
  HDErrorInfo error = hdGetError();
  // NOTE: The device_handle > 1024 is a "fix" suggested by the owners of OpenHaptics.
  // The reason this check is needed, even though it looks like a complete hack for someone
  // that is not familiar with the inner workings of OpenHaptics, is that sometimes
  // some internal states are not set correctly for certain versions of OpenHaptics
  // which makes it impossible to have a chain of deviceName to test to initialize.
  // If failing to initialize the first device then the next call to hdInitDevice will
  // not result in an error and the device_handle will be a seemingly high garbage number.
  if( HD_DEVICE_ERROR( error ) || device_handle > 1024 ) {
    std::stringstream s;
    if( device_name == "" )
      s << "Could not init default Phantom device. ";
    else
      s << "Could not init Phantom device named \"" << device_name << "\".";
    s << "Error code: ";
    s << std::string( hdGetErrorString( error.errorCode ) ) ;
    s << ". Internal error code: " << error.internalErrorCode;
    setErrorMsg( s.str() );
    return false;
  }
  hdMakeCurrentDevice( device_handle );

  HDdouble d;
  hdGetDoublev( HD_DEVICE_FIRMWARE_VERSION, &d );
  device_firmware_version = d;
  device_model_type = hdGetString( HD_DEVICE_MODEL_TYPE );
  device_driver_version = hdGetString( HD_DEVICE_DRIVER_VERSION );
  device_vendor = hdGetString( HD_DEVICE_VENDOR );
  device_serial_number = hdGetString( HD_DEVICE_SERIAL_NUMBER );

  hdGetDoublev( HD_NOMINAL_MAX_STIFFNESS, &d );
  max_stiffness = d * 1e3;

  HDdouble ws[6];
  hdGetDoublev( HD_USABLE_WORKSPACE_DIMENSIONS, ws );
  usable_workspace_min = 1e-3 * Vec3( ws[0], ws[1], ws[2] );
  usable_workspace_max = 1e-3 * Vec3( ws[3], ws[4], ws[5] );
  hdGetDoublev( HD_MAX_WORKSPACE_DIMENSIONS, ws );
  max_workspace_min = 1e-3 * Vec3( ws[0], ws[1], ws[2] );
  max_workspace_max = 1e-3 * Vec3( ws[3], ws[4], ws[5] );

  hdGetDoublev( HD_NOMINAL_MAX_FORCE, &d );
  max_force = d;

  hdGetDoublev( HD_NOMINAL_MAX_CONTINUOUS_FORCE, &d );
  max_cont_force = d;

  hdGetDoublev( HD_TABLETOP_OFFSET, &d );
  tabletop_offset = 1e-3 * d;

  HDint i;
  hdGetIntegerv( HD_INPUT_DOF, &i );
  input_dof = i;

  hdGetIntegerv( HD_OUTPUT_DOF, &i );
  output_dof = i;
  /*
  cerr << "HD API version: " << hdapi_version << endl;
  
  cerr << "Firmware version: " << device_firmware_version << endl;
  cerr << "Device model: " << device_model_type << endl;
  cerr << "Vendor: " << device_vendor << endl;
  cerr << "Driver version: " << device_driver_version << endl;
  cerr << "Serial number: " << device_serial_number << endl;
  cerr << "Max Ws max: " << max_workspace_max << endl;
  cerr << "Max Ws min: " << max_workspace_min << endl;
  cerr << "U Ws max: " << usable_workspace_max << endl;
  cerr << "U Ws min: " << usable_workspace_min << endl;

  cerr << "Max force: " << max_force << endl;
  cerr << "Max cont force: " << max_cont_force << endl;  
  cerr << "Offset: " << tabletop_offset << endl;
  cerr << "Input dof: " << input_dof << endl;
  cerr << "Output dof: " << output_dof << endl;
  */
  hdEnable(HD_FORCE_OUTPUT);

  // Add callbacks in order to do hdBeginFrame/hdEndFrame in order to only get
  // one pair of them per loop. Must be done in order to use hd api and hl api
  // together.
  HDCallbackCode handle = 
    hdScheduleAsynchronous( PhantomDeviceInternal::beginFrameCallback,
                            this,
                            HD_MAX_SCHEDULER_PRIORITY );
  hd_handles.push_back( handle );
  handle = hdScheduleAsynchronous( endFrameCallback,
                                   this,
                                   HD_MIN_SCHEDULER_PRIORITY );
  hd_handles.push_back( handle );

  HLThread *hl_thread = HLThread::getInstance();
  thread = hl_thread;
  ++nr_of_scheduled;
  if( !scheduler_started ) {
    // Just making sure that no other calls between the last check and this one
    // generated any errors.
    error = hdGetError();
    if( HD_DEVICE_ERROR( error ) ) {
      std::stringstream s;
      if( device_name == "" )
        s << "Could not init default Phantom device. ";
      else
        s << "Could not init Phantom device named \"" << device_name << "\".";
      s << "Error code: ";
      s << std::string( hdGetErrorString( error.errorCode ) ) ;
      s << ". Internal error code: " << error.internalErrorCode;
      setErrorMsg( s.str() );
      return false;
    }
    hdSetSchedulerRate( (HDulong)_thread_frequency );
    // Need to catch the error that might occur from trying to set the
    // scheduler rate to an invalid value. According to OpenHaptics reference
    // this should always be HD_INVALID_VALUE but this is not always
    // true. Sometimes it is a HD_TIMER_ERROR. There is no way to check
    // if the scheduler rate is a valid value before hand unless
    // we implement something for each SensAble model. HD_INVALID_OPERATION
    // gets returned by OpenHaptics 3.0 and will be circumvented as well.
    HDErrorInfo error2 = hdGetError();

    if( HD_DEVICE_ERROR( error2 ) && (
        error2.errorCode != HD_INVALID_VALUE &&
        error2.errorCode != HD_TIMER_ERROR &&
        error2.errorCode != HD_INVALID_OPERATION) ) {
      std::stringstream s;
      if( device_name == "" )
        s << "Could not init default Phantom device. ";
      else
        s << "Could not init Phantom device named \"" << device_name << "\".";
      s << "Error code: ";
      s << std::string( hdGetErrorString( error2.errorCode ) ) ;
      s << ". Internal error code: " << error2.internalErrorCode;
      setErrorMsg( s.str() );
      return false;
    }
  }
  return true;
}

bool PhantomHapticsDevice::releaseHapticsDevice() {
  HAPIHapticsDevice::disableDevice();

  --nr_of_scheduled;
  hdMakeCurrentDevice( device_handle );

  bool restart = true;
  if( !scheduler_started || ( scheduler_started && nr_of_scheduled == 0 ) )
    restart = false;
  stopScheduler();

  for( std::vector< HDCallbackCode >::iterator i = hd_handles.begin();
       i != hd_handles.end();
       ++i ) {
    hdUnschedule(*i);
  }
  hd_handles.clear();

  hdDisableDevice( device_handle );
  device_handle = 0;
  HLThread *hl_thread = static_cast< HLThread * >( thread );
  if( nr_of_scheduled == 0)
    hl_thread->setActive( false );
  thread = NULL;
  if( restart )
    startScheduler();
  return true;
}

void PhantomHapticsDevice::updateDeviceValues( DeviceValues &dv,
                                               HAPITime dt ) {
  HDErrorInfo error;
  error = hdGetError();
  if (HD_DEVICE_ERROR(error)) {
    if( error_handler.get() ) {
      error_handler->handleError( this,
                                  error.errorCode,
                                  hdGetErrorString(error.errorCode) );
    }
  }

  HAPIHapticsDevice::updateDeviceValues( dv, dt );
  hdMakeCurrentDevice( device_handle );
  HDdouble v[16];
  hdGetDoublev( HD_CURRENT_POSITION, v ); 
  dv.position = 1e-3 * Vec3( v[0], v[1], v[2] );
  hdGetDoublev( HD_CURRENT_VELOCITY, v ); 
  dv.velocity = 1e-3 * Vec3( v[0], v[1], v[2] );
  hdGetIntegerv( HD_CURRENT_BUTTONS, &dv.button_status );    
  hdGetDoublev( HD_CURRENT_TRANSFORM, v );
  dv.orientation = Rotation( Matrix3( v[0], v[4], v[8],
                                      v[1], v[5], v[9],
                                      v[2], v[6], v[10] ) );
  hdGetDoublev( HD_CURRENT_JOINT_ANGLES, v );
  joint_angles = Vec3( v[0], v[1], v[2] );
  hdGetDoublev( HD_CURRENT_GIMBAL_ANGLES, v );
  gimbal_angles = Vec3( v[0], v[1], v[2] );
  v[3] = v[4] = v[5] = 0;
  hdGetDoublev( HD_MOTOR_TEMPERATURE, v );
  motor_temperatures[0] = v[0];
  motor_temperatures[1] = v[1];
  motor_temperatures[2] = v[2];
  motor_temperatures[3] = v[3];
  motor_temperatures[4] = v[4];
  motor_temperatures[5] = v[5];
  
  HDlong vv[6];
  hdGetLongv( HD_CURRENT_ENCODER_VALUES, vv );
  encoder_values[0] = vv[0];
  encoder_values[1] = vv[1];
  encoder_values[2] = vv[2];
  encoder_values[3] = vv[3];
  encoder_values[4] = vv[4];
  encoder_values[5] = vv[5];
  
  HDboolean inkwell_switch;
  hdGetBooleanv( HD_CURRENT_INKWELL_SWITCH, &inkwell_switch );
  in_inkwell = !inkwell_switch;
}

void PhantomHapticsDevice::sendOutput( DeviceOutput &dv,
                                       HAPITime dt ) {
  hdMakeCurrentDevice( device_handle );
  HDdouble v[3];
  v[0] = dv.force.x;
  v[1] = dv.force.y;
  v[2] = dv.force.z;
  hdSetDoublev( HD_CURRENT_FORCE, v ); 

  // torque in OpenHaptics is in Nmm
  v[0] = dv.torque.x * 1e3;
  v[1] = dv.torque.y * 1e3;
  v[2] = dv.torque.z * 1e3;
  hdSetDoublev( HD_CURRENT_TORQUE, v ); 
  HDErrorInfo error;
  error = hdGetError();
  if (HD_DEVICE_ERROR(error)) {
    if( error_handler.get() ) {
      error_handler->handleError( this,
                                  error.errorCode,
                                  hdGetErrorString(error.errorCode) );
    }
  }
}

bool PhantomHapticsDevice::needsCalibration() {
  if( device_state != UNINITIALIZED ) {
    hdMakeCurrentDevice( device_handle );
    bool needs_calibration =
      hdCheckCalibration() != HD_CALIBRATION_OK;
    if( !needs_calibration && in_calibration_mode )
      in_calibration_mode = false;
    return needs_calibration;
  } else {
    in_calibration_mode = false;
    return false;
  }
}

bool PhantomHapticsDevice::calibrateDevice() {
  if( device_state != UNINITIALIZED ) {
    in_calibration_mode = true;
    return true;
  } else {
    return false;
  }
}

void PhantomHapticsDevice::calibrateDeviceInternal() {
  hdMakeCurrentDevice( device_handle );
  HDint style;
  hdGetIntegerv(HD_CALIBRATION_STYLE,&style);
  hdUpdateCalibration(style);
}

void PhantomHapticsDevice::startScheduler() {
  if( !scheduler_started ) {
    hdStartScheduler();
    HDErrorInfo err = hdGetError();
    if( HD_DEVICE_ERROR( err ) ) {
      std::cerr << "Error starting haptics scheduler: " << hdGetErrorString( err.errorCode ) << std::endl;
    } else {
      HLThread::getInstance()->setActive( true );
      scheduler_started = true;
    }
  }
}

void PhantomHapticsDevice::stopScheduler( bool call_hd ) {
  if( scheduler_started ) {
    if( call_hd )
      hdStopScheduler();
    scheduler_started = false;
  }
}

bool PhantomHapticsDevice::hardwareForceDisabled() {
  HDboolean inkwell_switch;
  hdGetBooleanv( HD_CURRENT_INKWELL_SWITCH, &inkwell_switch );
  HDboolean safety_switch;
  hdGetBooleanv( HD_CURRENT_SAFETY_SWITCH, &safety_switch );
  // Aparently inkwell_switch is 0 if the omni is inside the well.
  // NOTE: the safety switch parameter is untested, use at your own risk.
  return ( !inkwell_switch || safety_switch );
}

#endif
