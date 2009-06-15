//////////////////////////////////////////////////////////////////////////////
//    Copyright 2004-2007, SenseGraphics AB
//    Copyright 2008-2009, Kyle Machulis
//    Copyright 2009, Karljohan Lundin Palmerius
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
/// \file NiFalconHapticsDevice.cpp
/// \brief Cpp file for NiFalconHapticsDevice.
///
//
//////////////////////////////////////////////////////////////////////////////


#include <HAPI/NiFalconHapticsDevice.h>

#ifdef HAVE_NIFALCONAPI

#ifdef NIFALCON_LIBFTD2XX
# include <falcon/comm/FalconCommFTD2XX.h>
#endif

#ifdef NIFALCON_LIBFTDI
# include <falcon/comm/FalconCommLibFTDI.h>
#endif

#ifdef NIFALCON_LIBUSB
# include <falcon/comm/FalconCommLibUSB.h>
#endif

#include <falcon/firmware/FalconFirmwareNovintSDK.h>
#include <falcon/kinematic/FalconKinematicStamper.h>
#include <falcon/util/FalconFirmwareBinaryNvent.h>

#include <falcon/grip/FalconGripFourButton.h>

#include <H3DUtil/DynamicLibrary.h>

//#define NIFALCON_DEBUG

using namespace HAPI;
using namespace libnifalcon;


namespace NiFalconHapticsDeviceInternal {
  list< string > nifalcon_device_libs;
}

HAPIHapticsDevice::HapticsDeviceRegistration 
NiFalconHapticsDevice::device_registration
( "NiFalconHapticsDevice",
  &(newInstance< NiFalconHapticsDevice >),
  NiFalconHapticsDeviceInternal::nifalcon_device_libs );

bool NiFalconHapticsDevice::initHapticsDevice( int _thread_frequency ) {
  
#ifdef NIFALCON_LIBUSB
  H3DUtil::DynamicLibrary::load("libnifalcon_comm_libusb.so");
  device.setFalconComm<libnifalcon::FalconCommLibUSB>();
#else
#ifdef NIFALCON_LIBFTD2XX
  device.setFalconComm<libnifalcon::FalconCommLibFTD2XX>();
#else
#ifdef NIFALCON_LIBFTDI
  device.setFalconComm<libnifalcon::FalconCommLibFTDI>();
#endif
#endif
#endif
  
  device.setFalconFirmware<libnifalcon::FalconFirmwareNovintSDK>();
  device.setFalconKinematic<libnifalcon::FalconKinematicStamper>();
  device.setFalconGrip<libnifalcon::FalconGripFourButton>();
 
  // get number of connected falcons.
  unsigned int num_falcons;	
  if(!device.getDeviceCount(num_falcons)) {
#ifdef NIFALCON_DEBUG
    H3DUtil::Console(4) << "Cannot get device count" << std::endl;
#endif
  }
	
#ifdef NIFALCON_DEBUG
  H3DUtil::Console(4) << "Falcons found: " << num_falcons << std::endl;
  H3DUtil::Console(4) << "Opening falcon" << std::endl;
#endif
  
  // open device
  if(!device.open(0)) {
    stringstream s;
    s << "Could not init Falcon device(libnifalcon). ";
    s << "Error code: ";
    s << device.getErrorCode();
    setErrorMsg( s.str() );
    return false;
  }

#ifdef NIFALCON_DEBUG
   H3DUtil::Console(4) << "Opened falcon" << std::endl;
#endif

   // try to load firmware
   bool has_firmware = false;
   for(int i = 0; i < 10; ++i) {
     if(!device.getFalconFirmware()->loadFirmware(true, NOVINT_FALCON_NVENT_FIRMWARE_SIZE, const_cast<uint8_t*>(NOVINT_FALCON_NVENT_FIRMWARE))) {
#ifdef NIFALCON_DEBUG
       H3DUtil::Console(4) << "Cannot load firmware" << std::endl;
#endif
     } else {
       has_firmware = true;
#ifdef NIFALCON_DEBUG
       H3DUtil::Console(4) <<"firmware loaded" << std::endl;
#endif
       break;
     }
  }

  // no firmware could be loaded
  if(!has_firmware) {
    setErrorMsg( "Could not init Falcon device(libnifalcon). Could not load firmware" );
    return false;
  }


  // set up a communication thread for the device.
  com_thread = 
    new H3DUtil::PeriodicThread( H3DUtil::ThreadBase::HIGH_PRIORITY, 1000 );
  com_thread->setThreadName( "NiFalconHapticsDevice com thread" );
  
  com_func_cb_handle = com_thread->asynchronousCallback( com_func, this );

  return true;
}

bool NiFalconHapticsDevice::releaseHapticsDevice() {
  HAPIHapticsDevice::disableDevice();
  
  if( com_thread ) {
    if( com_func_cb_handle != -1 ) {
      com_thread->removeAsynchronousCallback( com_func_cb_handle );
      com_func_cb_handle = -1;
    }
    delete com_thread;
    com_thread = NULL;
  }
  
  return true;
}

void NiFalconHapticsDevice::updateDeviceValues( DeviceValues &dv,
                                                HAPITime dt ) {
  HAPIHapticsDevice::updateDeviceValues( dv, dt );
  com_lock.lock();
  dv.position = current_values.position;
  calculateVelocity(dv, dt);
  dv.orientation = current_values.orientation;
  dv.button_status = current_values.button_status;
  com_lock.unlock();
}

void NiFalconHapticsDevice::sendOutput( DeviceOutput &dv,
                                        HAPITime dt ) {
  com_lock.lock();
  current_values.force = dv.force;
  current_values.torque = dv.torque;
  com_lock.unlock();
}

H3DUtil::PeriodicThread::CallbackCode
NiFalconHapticsDevice::com_func( void *data ) {
  NiFalconHapticsDevice *hd = 
    static_cast< NiFalconHapticsDevice * >( data );
  
  if( hd->getDeviceState() != HAPIHapticsDevice::ENABLED ) {
    return H3DUtil::PeriodicThread::CALLBACK_CONTINUE; }
  
  hd->device.getFalconFirmware()->setHomingMode( true );
  hd->device.runIOLoop();
  if( hd->device.getFalconFirmware()->isHomed() ) {

    // get position
    boost::array<double,3> pos = hd->device.getPosition();
    Vec3 position( pos[0] , pos[1] , (pos[2] - .150f) );
    
    HAPIInt32 button_status;
    
    // get buttons
    boost::shared_ptr<FalconGrip> grip = hd->device.getFalconGrip();
    for( int i = 0 ; i < grip->getNumDigitalInputs() ; i++ ){
      if( grip->getDigitalInput(i) ){
        button_status |= 0x01 << i; }
    }
    
    Rotation orientation;
    
    // transfer values to/from haptics thread.
    hd->com_lock.lock();
    
    hd->current_values.position = position;
    hd->current_values.button_status = button_status;
    hd->current_values.orientation = orientation;

    boost::array<double,3> f;
    f[0] = hd->current_values.force.x;
    f[1] = hd->current_values.force.y;
    f[2] = hd->current_values.force.z;
     
    hd->com_lock.unlock();
 
    // send force
    hd->device.setForce(f);
 }
  
  return H3DUtil::PeriodicThread::CALLBACK_CONTINUE;
}

#endif
