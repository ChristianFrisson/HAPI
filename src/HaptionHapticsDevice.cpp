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
/// \file HaptionHapticsDevice.cpp
/// \brief Cpp file for HaptionHapticsDevice.
///
//
//////////////////////////////////////////////////////////////////////////////



#include <HAPI/HaptionHapticsDevice.h>
#include <H3DUtil/DynamicLibrary.h>

#ifdef HAVE_VIRTUOSEAPI

using namespace HAPI;

namespace HaptionHapticsDeviceInternal {
  std::string libs_array[1] = {"VirtuoseDLL.dll"};
  std::list< std::string > haption_libs(libs_array, libs_array + 1 );
}

HAPIHapticsDevice::HapticsDeviceRegistration 
HaptionHapticsDevice::device_registration(
                            "Haption",
                            &(newInstance< HaptionHapticsDevice >),
                            HaptionHapticsDeviceInternal::haption_libs
                            );


bool HaptionHapticsDevice::initHapticsDevice( int _thread_frequency ) {
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
#endif //H3D_WINDOWS
  context = virtOpen( ip_address.c_str() );
  if (context == NULL) {
    int error_code = virtGetErrorCode(NULL);
    std::stringstream s;
    s << "Warning: Failed to open Virtuose device.";
    if( error_code != VIRT_E_VIRTUOSE_DLL_NOT_FOUND )
      s << " Error: " << virtGetErrorMessage(error_code) << std::endl;
    else
      s << " Could not load library.";
    setErrorMsg( s.str() );
    return false;
  }

  float identity[7] = {0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,1.0f};
  virtSetIndexingMode( context, INDEXING_ALL);
  virtSetForceFactor( context, 1.0f);
  virtSetSpeedFactor( context, 1.0f);
  virtSetTimeStep( context, 0.003f);
  virtSetBaseFrame( context, identity);
  virtSetObservationFrame( context, identity);
  virtSetCommandType( context, COMMAND_TYPE_IMPEDANCE );
  virtSetPowerOn( context, 1);

  com_thread = 
    new H3DUtil::PeriodicThread( H3DUtil::ThreadBase::HIGH_PRIORITY, 1000 );
  com_thread->setThreadName( "Virtuose com thread" );

  com_func_cb_handle = com_thread->asynchronousCallback( com_func, this );

  return true;
}

bool HaptionHapticsDevice::releaseHapticsDevice() {
  HAPIHapticsDevice::disableDevice();
  if( context != NULL ) {
    if( com_thread ) {
      if( com_func_cb_handle != -1 ) {
        com_thread->removeAsynchronousCallback( com_func_cb_handle );
        com_func_cb_handle = -1;
      }
      delete com_thread;
      com_thread = NULL;
    }

    virtClose( context );
    context = NULL;
  }
  return true;
}


void HaptionHapticsDevice::updateDeviceValues( DeviceValues &dv,
                                               HAPITime dt ) {
  HAPIHapticsDevice::updateDeviceValues( dv, dt );

  if( context != NULL ) {
    com_lock.lock();
    dv.position = current_values.position;
    dv.orientation = current_values.orientation;
    dv.velocity = current_values.velocity;
    dv.button_status = current_values.button_status;
    com_lock.unlock();
  }
}

void HaptionHapticsDevice::sendOutput( DeviceOutput &dv,
                                   HAPITime dt ) {
  if( context != NULL ) {
    com_lock.lock();
    current_values.force = dv.force;
    current_values.torque = dv.torque;
    com_lock.unlock();
  }
}


H3DUtil::PeriodicThread::CallbackCode
HaptionHapticsDevice::com_func( void *data ) {
  HaptionHapticsDevice *hd = 
    static_cast< HaptionHapticsDevice * >( data );
  
  if( hd->context != NULL ) {
    float v[7], s[6];
    int b0, b1;
    virtGetPosition( hd->context, v );
    virtGetSpeed( hd->context, s );
    virtGetButton( hd->context, 1, &b0 );
    virtGetButton( hd->context, 2, &b1 );

    HAPIInt32 button = (b1 << 1) | b0;
    Vec3 position = Vec3( v[1], v[2], v[0] );
    Vec3 velocity = Vec3( s[1], s[2], s[0] );

    // The handle of the device is along the y-axis, so we rotate it to match
    Rotation orientation( Quaternion( v[4], v[5], v[3], v[6] )*Rotation(1,0,0,-H3DUtil::Constants::pi/2 ));

    hd->com_lock.lock();

    hd->current_values.position = position;
    hd->current_values.velocity = velocity;
    hd->current_values.button_status = button;
    hd->current_values.orientation = orientation;
 
    Vec3 force = hd->current_values.force;
    Vec3 torque = hd->current_values.torque;
    hd->com_lock.unlock();

    float f[6] = { (float)force.z, (float)force.x, (float)force.y,
                   (float)torque.z, (float)torque.x, (float)torque.y };
    virtSetForce( hd->context, f );
  }
  return H3DUtil::PeriodicThread::CALLBACK_CONTINUE;
}


#endif
