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
/// \file FalconHapticsDevice.cpp
/// \brief Cpp file for FalconHapticsDevice.
///
//
//////////////////////////////////////////////////////////////////////////////


#include <HAPI/FalconHapticsDevice.h>
#include <H3DUtil/DynamicLibrary.h>

#ifdef HAVE_FALCONAPI

using namespace HAPI;
using namespace std;

namespace FalconHapticsDeviceInternal {
  string libs_array[1] = {"hdl.dll"};
  list< string > falcon_device_libs(libs_array, libs_array + 1 );
}

HAPIHapticsDevice::HapticsDeviceRegistration 
FalconHapticsDevice::device_registration(
                            "Falcon",
                            &(newInstance< FalconHapticsDevice >),
                            FalconHapticsDeviceInternal::falcon_device_libs
                            );

bool FalconHapticsDevice::hdl_started = false;
int FalconHapticsDevice::nr_of_initalized = 0;

bool FalconHapticsDevice::initHapticsDevice( int _thread_frequency ) {
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
  if( device_name != "" ) {
    device_handle = hdlInitNamedDevice( device_name.c_str() );
  } else {
    if( device_index < hdlCountDevices() ) {
      device_handle = hdlInitIndexedDevice( device_index );
    } else {
      // hdl crashes if I call the hdlGetError without calling any init function before it. So we
      // have a special return for it here before that call.
      stringstream s;
      s << "Could not init Falcon device. ";
      if( device_index == 0 ) s << "Not enough devices connected." << endl;
      else {
        s << hdlCountDevices() << " devices connected, but trying to initialize "
          << "device with index " << device_index << "." <<endl;  
      }
      setErrorMsg( s.str() );
      return false;      
    }
  }
  
  HDLError error = hdlGetError();
  if ( device_handle == -1 || error != HDL_NO_ERROR ) {
    stringstream s;
    s << "Could not init Falcon device. ";
    if( error != HDL_NO_ERROR ) 
      s << "Error code: " << error << endl;
     
    setErrorMsg( s.str() );
    return false;
  }
  
  ++nr_of_initalized;
  hdlMakeCurrent( device_handle );

  double ws[6];
  hdlDeviceWorkspace( ws );
  workspace_min = Vec3( ws[0], ws[1], ws[1] );
  workspace_max = Vec3( ws[3], ws[4], ws[5] );

  device_model = hdlDeviceModel();
  
  FalconThread *fl_thread = FalconThread::getInstance();
  thread = fl_thread;
  if( hdl_started )
    hdlStop();
  hdlStart();
  hdl_started = true;
  fl_thread->setActive( true );
  
  return true;
}

bool FalconHapticsDevice::releaseHapticsDevice() {
  HAPIHapticsDevice::disableDevice();

  hdlMakeCurrent( device_handle );

  hdlUninitDevice( device_handle );
  --nr_of_initalized;
  if( nr_of_initalized == 0 ) {
    hdlStop();
  }
  device_handle = 0;

  FalconThread *fl_thread = static_cast< FalconThread * >( thread );
  fl_thread->setActive( false );
  thread = NULL;
  return true;
}

void FalconHapticsDevice::updateDeviceValues( DeviceValues &dv,
                                              HAPITime dt ) {
  checkIfHDALError();
 
  HAPIHapticsDevice::updateDeviceValues( dv, dt );
  hdlMakeCurrent( device_handle );

  double v[16];
  hdlToolPosition( v ); 
  dv.position = Vec3( v[0], v[1], v[2] );

  calculateVelocity( dv, dt );

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

  checkIfHDALError();
}

namespace FalconThreadInternals {
  H3DUtil::MutexLock callback_handles_lock;
  typedef std::list< std::pair< int, std::pair< void *, HDLOpHandle > > >
    CallbackHandleList;
  CallbackHandleList callback_handles;
}

HDLServoOpExitCode falconCallbackSynchronous( void *_data ) {
  void * * data = static_cast< void * * >( _data );
  FalconHapticsDevice::FalconThread::CallbackFunc* func = 
    static_cast< FalconHapticsDevice::FalconThread::CallbackFunc* >( _data );
  //FalconThread::CallbackFunc func = static_cast< FalconThread::CallbackFunc >( data[0] );
  FalconHapticsDevice::FalconThread::CallbackCode c = (*func)( data[1] );
  if( c == FalconHapticsDevice::FalconThread::CALLBACK_DONE )
    return HDL_SERVOOP_EXIT;
  else if( c == FalconHapticsDevice::FalconThread::CALLBACK_CONTINUE )
    return HDL_SERVOOP_CONTINUE;
  else return c;
}

HDLServoOpExitCode falconCallbackAsynchronous( void *_data ) {
  void * * data = static_cast< void * * >( _data );
  FalconHapticsDevice::FalconThread::CallbackFunc* func = 
    static_cast< FalconHapticsDevice::FalconThread::CallbackFunc* >( _data );
  //FalconThread::CallbackFunc func = static_cast< FalconThread::CallbackFunc >( data[0] );
  FalconHapticsDevice::FalconThread::CallbackCode c = (*func)( data[1] );
  if( c == FalconHapticsDevice::FalconThread::CALLBACK_DONE ) {
    FalconThreadInternals::callback_handles_lock.lock();
    for( FalconThreadInternals::CallbackHandleList::iterator i =
         FalconThreadInternals::callback_handles.begin();
         i != FalconThreadInternals::callback_handles.end(); ++i ) {
      if( (*i).second.first == _data ) {
        FalconHapticsDevice::FalconThread * falc_thread =
          static_cast< FalconHapticsDevice::FalconThread * >(data[2]);
        falc_thread->addFreeId( (*i).first );
        FalconThreadInternals::callback_handles.erase( i );
        break;
      }
    }
    FalconThreadInternals::callback_handles_lock.unlock();
    delete [] data;
    return HDL_SERVOOP_EXIT;
  } else if( c == FalconHapticsDevice::FalconThread::CALLBACK_CONTINUE )
    return HDL_SERVOOP_CONTINUE;
  else return c;
}

void FalconHapticsDevice::FalconThread::synchronousCallback( 
                CallbackFunc func, void *data ) {
  void * param[] = { (void*)func, data };
  hdlCreateServoOp( falconCallbackSynchronous,
                    param,
                    true );
}

int FalconHapticsDevice::FalconThread
  ::asynchronousCallback( CallbackFunc func, void *data ) {
  void * * param = new void * [3];
  param[0] = (void*)func;
  param[1] = data;
  param[2] = (void*)this;
  HDLOpHandle falcon_cb_handle =
    hdlCreateServoOp( falconCallbackAsynchronous,
                      param,
                      false );
  FalconThreadInternals::callback_handles_lock.lock();
  int cb_handle = genCallbackId();
  FalconThreadInternals::callback_handles.push_back(
    std::make_pair( cb_handle, std::make_pair( param, falcon_cb_handle ) ) );
  FalconThreadInternals::callback_handles_lock.unlock();
  return cb_handle;
}

bool FalconHapticsDevice::FalconThread::
  removeAsynchronousCallback( int callback_handle ) {
  FalconThreadInternals::callback_handles_lock.lock();
  for( FalconThreadInternals::CallbackHandleList::iterator i =
         FalconThreadInternals::callback_handles.begin();
       i != FalconThreadInternals::callback_handles.end(); ++i ) {
    if( (*i).first == callback_handle ) {
      free_ids.push_back( callback_handle );
      hdlDestroyServoOp( (*i).second.second );
      void * data = (*i).second.first;
      FalconThreadInternals::callback_handles.erase( i );
      // Delete param that was allocated in asyncronous callback.
      delete [] data;
      FalconThreadInternals::callback_handles_lock.unlock();
      return true;
    }
  }
  FalconThreadInternals::callback_handles_lock.unlock();
  return false;
}

H3DUtil::PeriodicThread::CallbackCode FalconHapticsDevice::FalconThread
  ::setThreadId( void * data ) {
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

void FalconHapticsDevice::checkIfHDALError() {
  HDLError error;
  error = hdlGetError();
  if ( error != HDL_NO_ERROR ) {
    if( error_handler.get() ) {
      string error_string = "";
      switch (error) {
        case HDL_ERROR_INTERNAL: {
          error_string = "HDAL internal error.";
          break;
        }
        case HDL_ERROR_STACK_OVERFLOW: {
          error_string = "Overflow of error stack.";
          break;
        }
        case HDL_SERVO_START_ERROR: {
          error_string = "Could not start servo thread.";
          break;
        }
        // The following error codes are all initializations errors and
        // should therefore never happen here, but we put them here just
        // 
        case HDL_ERROR_INIT_FAILED: {
          error_string = "Device initialization error.";
          break;
        }
        case HDL_INIT_DEVICE_ALREADY_INITED: {
          error_string = "Device already initalized.";
          break;
        }
        case HDL_INIT_DEVICE_FAILURE: {
          error_string = "Failed to initalize device.";
          break;
        }
        case HDL_INIT_DEVICE_NOT_CONNECTED: {
          error_string = "Requested device not connected.";
          break;
        }
        case HDL_INIT_DLL_LOAD_ERROR: {
          error_string = "Could not load driver DLL.";
          break;
        }
        case HDL_INIT_ERROR_MASK: {
          error_string = "Mask for all initialization errors.";
          break;
        }
        case HDL_INIT_INI_DLL_STRING_NOT_FOUND: {
          error_string = "No DLL name in configuration file.";
          break;
        }
        case HDL_INIT_INI_MANUFACTURER_NAME_STRING_NOT_FOUND: {
          error_string = "No MANUFACTURER_NAME value in configuration file.";
          break;
        }
        case HDL_INIT_INI_NOT_FOUND: {
          error_string = "Could not find configuration file.";
          break;
        }
        default: {
          error_string = "No string for this error, check error code.";
        }
      }
      error_handler->handleError( this,
                                  error,
                                  error_string );
    }
  }
}


#endif
