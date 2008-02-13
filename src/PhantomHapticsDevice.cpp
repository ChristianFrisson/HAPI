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
/// \file PhantomHapticsDevice.cpp
/// \brief Cpp file for PhantomHapticsDevice.
///
//
//////////////////////////////////////////////////////////////////////////////


#include <HAPI/PhantomHapticsDevice.h>

#ifdef HAVE_OPENHAPTICS
#include <HAPI/HLThread.h>
#include <sstream>

using namespace HAPI;

#ifdef _MSC_VER
#pragma comment( lib, "hd.lib" )
#endif

namespace PhantomDeviceInternal {
  string libs_array[1] = {"HD.dll"};
  list< string > phantom_device_libs(libs_array, libs_array + 1 );

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
  HAPIHapticsDevice::hapticRenderingCallback(
    hd->haptic_rendering_callback_data );
  hdEndFrame( hd->getDeviceHandle() );
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
  device_handle = hdInitDevice( device_name == "" ? 
                                HD_DEFAULT_DEVICE : device_name.c_str() );
  HDErrorInfo error = hdGetError();
  if ( HD_DEVICE_ERROR( error ) ) {
    stringstream s;
    s << "Could not init Phantom device. Error code: "
      << string( hdGetErrorString( error.errorCode ) ) 
      << ". Internal error code: " << error.internalErrorCode;
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
  max_stiffness = d;

  HDdouble ws[6];
  hdGetDoublev( HD_USABLE_WORKSPACE_DIMENSIONS, ws );
  usable_workspace_min = Vec3( ws[0], ws[1], ws[1] );
  usable_workspace_max = Vec3( ws[3], ws[4], ws[5] );
  hdGetDoublev( HD_MAX_WORKSPACE_DIMENSIONS, ws );
  max_workspace_min = Vec3( ws[0], ws[1], ws[1] );
  max_workspace_max = Vec3( ws[3], ws[4], ws[5] );

  hdGetDoublev( HD_NOMINAL_MAX_FORCE, &d );
  max_force = d;

  hdGetDoublev( HD_NOMINAL_MAX_CONTINUOUS_FORCE, &d );
  max_cont_force = d;

  hdGetDoublev( HD_TABLETOP_OFFSET, &d );
  tabletop_offset = d;

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
  hl_thread->setActive( true );
  nr_of_scheduled++;
  if( !scheduler_started )
    hdSetSchedulerRate( (HDulong)_thread_frequency );
  return true;
}

bool PhantomHapticsDevice::releaseHapticsDevice() {
  HAPIHapticsDevice::disableDevice();

  nr_of_scheduled--;
  hdMakeCurrentDevice( device_handle );

  if( scheduler_started && nr_of_scheduled == 0 ) {
    hdStopScheduler();
    scheduler_started = false;
  }

  for( vector< HDCallbackCode >::iterator i = hd_handles.begin();
       i != hd_handles.end();
       i++ ) {
    hdUnschedule(*i);
  }

  hdDisableDevice( device_handle );
  device_handle = 0;
  HLThread *hl_thread = static_cast< HLThread * >( thread );
  if( nr_of_scheduled == 0)
    hl_thread->setActive( false );
  thread = NULL;
  return true;
}

void PhantomHapticsDevice::updateDeviceValues( DeviceValues &dv,
                                               HAPITime dt ) {
  HDErrorInfo error;
  error = hdGetError();
  if (HD_DEVICE_ERROR(error))
    // TODO: do error handling
    cerr << hdGetErrorString(error.errorCode) << endl;
  HAPIHapticsDevice::updateDeviceValues( dv, dt );
  hdMakeCurrentDevice( device_handle );
  HDdouble v[16];
  hdGetDoublev( HD_CURRENT_POSITION, v ); 
  dv.position = Vec3( v[0], v[1], v[2] );
  hdGetDoublev( HD_CURRENT_VELOCITY, v ); 
  dv.velocity = Vec3( v[0], v[1], v[2] );
  hdGetIntegerv( HD_CURRENT_BUTTONS, &dv.button_status );    
  hdGetDoublev( HD_CURRENT_TRANSFORM, v );
  dv.orientation = Rotation( Matrix3( v[0], v[4], v[8],
                                       v[1], v[5], v[9],
                                       v[2], v[6], v[10] ) );
  hdGetDoublev( HD_CURRENT_JOINT_ANGLES, v );
  joint_angles = Vec3( v[0], v[1], v[2] );
  hdGetDoublev( HD_CURRENT_GIMBAL_ANGLES, v );
  gimbal_angles = Vec3( v[0], v[1], v[2] );
}

void PhantomHapticsDevice::sendOutput( DeviceOutput &dv,
                                       HAPITime dt ) {
  hdMakeCurrentDevice( device_handle );
  HDdouble v[3];
  v[0] = dv.force.x;
  v[1] = dv.force.y;
  v[2] = dv.force.z;
  hdSetDoublev( HD_CURRENT_FORCE, v ); 
  v[0] = dv.torque.x;
  v[1] = dv.torque.y;
  v[2] = dv.torque.z;
  hdSetDoublev( HD_CURRENT_TORQUE, v ); 
  HDErrorInfo error;
  error = hdGetError();
  if (HD_DEVICE_ERROR(error))
    // TODO: do error handling
    cerr << hdGetErrorString(error.errorCode) << endl;
}

bool PhantomHapticsDevice::needsCalibration() {
  if( device_state == INITIALIZED ) {
    hdMakeCurrentDevice( device_handle );
    return hdCheckCalibration() != HD_CALIBRATION_OK;
  } else {
    return false;
  }
}

bool PhantomHapticsDevice::calibrateDevice() {
  if( device_state == INITIALIZED ) {
    hdMakeCurrentDevice( device_handle );
    HDint style;
    hdGetIntegerv(HD_CALIBRATION_STYLE,&style);
    while(hdCheckCalibration()==HD_CALIBRATION_NEEDS_UPDATE) {
      hdUpdateCalibration(style);
    }
    return true;
  } else {
    return false;
  }
}

void PhantomHapticsDevice::startScheduler() {
  hdStartScheduler();
  scheduler_started = true;
}

#endif
