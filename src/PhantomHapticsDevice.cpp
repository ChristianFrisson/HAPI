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
/// \file PhantomHapticsDevice.cpp
/// \brief Cpp file for PhantomHapticsDevice.
///
//
//////////////////////////////////////////////////////////////////////////////

#include <PhantomHapticsDevice.h>
#include <sstream>

using namespace H3D;

#ifdef HAVE_OPENHAPTICS
#ifdef _MSC_VER
#pragma comment( lib, "hd.lib" )
#endif
#endif

bool PhantomHapticsDevice::initHapticsDevice() {
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
    
  hdEnable(HD_FORCE_OUTPUT);
    

  hdStartScheduler();

  HLThread *hl_thread = HLThread::getInstance();

  thread = hl_thread;
  hl_thread->setActive( true );
  
  /*
    HLint tmp_int;
    hdGetIntegerv( HD_INPUT_DOF, &tmp_int );
    inputDOF->setValue( tmp_int, id );
    
    hdGetIntegerv( HD_OUTPUT_DOF, &tmp_int );
    outputDOF->setValue( tmp_int, id );
  */
  return true;
}

bool PhantomHapticsDevice::releaseHapticsDevice() {
  HAPIHapticsDevice::disableDevice();
  //hdStopScheduler();
  hdDisableDevice( device_handle );
  device_handle = 0;
  HLThread *hl_thread = static_cast< HLThread * >( thread );
  hl_thread->setActive( false );
  thread = NULL;
  return true;
}

void PhantomHapticsDevice::updateDeviceValues( DeviceValues &dv,
                                               H3DTime dt ) {
  HDErrorInfo error;
  error = hdGetError();
  if (HD_DEVICE_ERROR(error))
    // do error handling
    cerr << hdGetErrorString(error.errorCode) << endl;
  hdBeginFrame( device_handle );
  HAPIHapticsDevice::updateDeviceValues( dv, dt );
  HDdouble v[16];
  hdGetDoublev( HD_CURRENT_POSITION, v ); 
  dv.position = Vec3d( v[0], v[1], v[2] );
  hdGetDoublev( HD_CURRENT_VELOCITY, v ); 
  dv.velocity = Vec3d( v[0], v[1], v[2] );
  hdGetIntegerv( HD_CURRENT_BUTTONS, &dv.button_status );    
  hdGetDoublev( HD_CURRENT_TRANSFORM, v );
  dv.orientation = Rotation( Matrix3d( v[0], v[4], v[8],
                                       v[1], v[5], v[9],
                                       v[2], v[6], v[10] ) );
}

void PhantomHapticsDevice::sendOutput( DeviceOutput &dv,
                                       H3DTime dt ) {
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
  hdEndFrame( device_handle );
  HDErrorInfo error;
  error = hdGetError();
  if (HD_DEVICE_ERROR(error))
    // do error handling
    cerr << hdGetErrorString(error.errorCode) << endl;
  HDdouble force[3];
  hdGetDoublev( HD_CURRENT_FORCE, force );
}

