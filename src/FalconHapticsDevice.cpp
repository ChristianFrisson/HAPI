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
/// \file FalconHapticsDevice.cpp
/// \brief Cpp file for FalconHapticsDevice.
///
//
//////////////////////////////////////////////////////////////////////////////


#include <FalconHapticsDevice.h>

#ifdef HAVE_FALCONAPI

using namespace HAPI;

#ifdef _MSC_VER
#pragma comment( lib, "hdl.lib" )
#endif

namespace FalconHapticsDeviceInternal {
  string libs_array[1] = {"hdl.dll"};
  list< string > falcon_device_libs(libs_array, libs_array + 1 );
  /*
  // Callback function that starts a new hd frame. It is used in order to 
  // encapsulate all HD API callback function within a hdBeginFrame/hdEndFrame
  // pair in order to only get one frame per loop.
  HDCallbackCode HDCALLBACK beginFrameCallback( void *data ) {
    FalconHapticsDevice *hd = static_cast< FalconHapticsDevice * >( data );
    hdBeginFrame( hd->getDeviceHandle() );
    return HD_CALLBACK_CONTINUE;
  }

  // Callback function that ends a hd frame. It is used in order to 
  // encapsulate all HD API callback function within a hdBeginFrame/hdEndFrame
  // pair in order to only get one frame per loop.
  HDCallbackCode HDCALLBACK endFrameCallback( void *data ) {
    FalconHapticsDevice *hd = static_cast< FalconHapticsDevice * >( data );
    hdEndFrame( hd->getDeviceHandle() );
    return HD_CALLBACK_CONTINUE;
  }
  */
}

HAPIHapticsDevice::HapticsDeviceRegistration 
FalconHapticsDevice::device_registration(
                            "Falcon",
                            &(newInstance< FalconHapticsDevice >),
                            FalconHapticsDeviceInternal::falcon_device_libs
                            );

bool FalconHapticsDevice::initHapticsDevice() {
  device_handle = hdlInitDevice( HDL_DEFAULT_DEVICE_ID );
	//device_handle = hdlInitNamedDevice( device_name.c_str() );
  HDLError error = hdlGetError();
  if ( device_handle == -1 || error != HDL_NO_ERROR ) {
    stringstream s;
    s << "Could not init Falcon device. ";
	if( error != HDL_NO_ERROR ) 
	  s << "Error code: " << error << endl;
     
    setErrorMsg( s.str() );
    return false;
  }   
  hdlMakeCurrent( device_handle );

  double ws[6];
  hdlDeviceWorkspace( ws );
  workspace_min = Vec3( ws[0], ws[1], ws[1] );
  workspace_max = Vec3( ws[3], ws[4], ws[5] );

  device_model = hdlDeviceModel();
  
  cerr << "Device model: " << device_model << endl;
  cerr << "Ws max: " << workspace_max << endl;
  cerr << "Ws min: " << workspace_min << endl;

  // TODO: threads

  //HLThread *hl_thread = HLThread::getInstance();
  //  thread = hl_thread;
  //  hl_thread->setActive( true );
  
  return true;
}

bool FalconHapticsDevice::releaseHapticsDevice() {
  HAPIHapticsDevice::disableDevice();

  hdlMakeCurrent( device_handle );

  /*  for( vector< HDCallbackCode >::iterator i = hd_handles.begin();
       i != hd_handles.end();
       i++ ) {
    hdUnschedule(*i);
    }*/

  // TODO: when is the servo loop stopped.
  //  hdStopScheduler();
  hdlUninitDevice( device_handle );
  device_handle = 0;

  // TODO:
  //  HLThread *hl_thread = static_cast< HLThread * >( thread );
  //  hl_thread->setActive( false );
  //  thread = NULL;
  return true;
}

void FalconHapticsDevice::updateDeviceValues( DeviceValues &dv,
                                              HAPITime dt ) {
  HDLError error;
  error = hdlGetError();
  if ( error != HDL_NO_ERROR )
    // TODO: do error handling
	cerr << "Device error: " << error << endl;
  HAPIHapticsDevice::updateDeviceValues( dv, dt );
  hdlMakeCurrent( device_handle );

  double v[16];
  hdlToolPosition( v ); 
  dv.position = 1e3 * Vec3( v[0], v[1], v[2] );

  // TODO: calculate velocity
  //hdGetDoublev( HD_CURRENT_VELOCITY, v ); 
  //dv.velocity = Vec3( v[0], v[1], v[2] );

  hdlToolOrientation_SQ( v );
  dv.orientation = Rotation( Matrix3( v[0], v[4], v[8],
                                      v[1], v[5], v[9],
                                      v[2], v[6], v[10] ) );

  hdlToolButtons(&dv.button_status );    
}

void FalconHapticsDevice::sendOutput( DeviceOutput &dv,
                                       HAPITime dt ) {
  hdlMakeCurrent( device_handle );

  double v[3];
  v[0] = dv.force.x;
  v[1] = dv.force.y;
  v[2] = dv.force.z;
  hdlSetToolForce( v ); 

  HDLError error;
  error = hdlGetError();
  if (error != HDL_NO_ERROR )
    // TODO: do error handling
    cerr << "Device error: " << error << endl;

}


auto_ptr< FalconHapticsDevice::FalconThread > 
FalconHapticsDevice::FalconThread::singleton( new FalconThread );

HDLServoOpExitCode falcon_callback( void *_data ) {
  void * * data = static_cast< void * * >( _data );
  FalconHapticsDevice::FalconThread::CallbackFunc* func = 
    static_cast< FalconHapticsDevice::FalconThread::CallbackFunc* >( _data );
  //FalconThread::CallbackFunc func = static_cast< FalconThread::CallbackFunc >( data[0] );
  FalconHapticsDevice::FalconThread::CallbackCode c = (*func)( data[1] );
  if( c == FalconHapticsDevice::FalconThread::CALLBACK_DONE ) return HDL_SERVOOP_EXIT;
  else if( c == FalconHapticsDevice::FalconThread::CALLBACK_CONTINUE ) return HDL_SERVOOP_EXIT;
  else return c;
}

void FalconHapticsDevice::FalconThread::synchronousCallback( 
                CallbackFunc func, void *data ) {
  void * param[] = { (void*)func, data };
  hdlCreateServoOp( falcon_callback,
                    param,
                    true );
}

void FalconHapticsDevice::FalconThread::asynchronousCallback( CallbackFunc func, void *data ) {
  void * * param = new void * [2];
  param[0] = (void*)func;
  param[1] = data;
  hdlCreateServoOp( falcon_callback,
                    param,
                    false );
}

H3DUtil::PeriodicThread::CallbackCode FalconHapticsDevice::FalconThread::setThreadId( void * data ) {
  FalconHapticsDevice::FalconThread *thread = 
	  static_cast< FalconHapticsDevice::FalconThread * >( data );
  thread->thread_id = H3DUtil::PeriodicThread::getCurrentThreadId();
  return H3DUtil::PeriodicThread::CALLBACK_DONE;
}

void FalconHapticsDevice::FalconThread::setActive( bool _active ) { 
  if( _active && !is_active ) {
    sg_lock.lock(); 
    threads.push_back( this );
    sg_lock.unlock();
    asynchronousCallback( setThreadId, this );
  } else if( !_active && is_active ) {
    sg_lock.lock();
    std::vector< H3DUtil::HapticThreadBase *>::iterator i = 
      std::find( threads.begin(), 
                 threads.end(), 
                 this );
    if( i != threads.end() ) {
      threads.erase( i );
    }
    sg_lock.unlock();
  }
  is_active = _active; 
}


#endif
