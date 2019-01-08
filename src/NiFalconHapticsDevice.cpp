//////////////////////////////////////////////////////////////////////////////
//    Copyright 2009-2019, SenseGraphics AB
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

#include <falcon/core/FalconDevice.h>

#ifdef NIFALCONAPI_LIBFTD2XX
# include <falcon/comm/FalconCommFTD2XX.h>
#endif

#ifdef NIFALCONAPI_LIBFTDI
# include <falcon/comm/FalconCommLibFTDI.h>
#endif

#ifdef NIFALCONAPI_LIBUSB
# include <falcon/comm/FalconCommLibUSB.h>
#endif

#include <falcon/firmware/FalconFirmwareNovintSDK.h>
#include <falcon/kinematic/FalconKinematicStamper.h>
#include <falcon/util/FalconFirmwareBinaryNvent.h>

#include <falcon/grip/FalconGripFourButton.h>

#include <H3DUtil/DynamicLibrary.h>


using namespace HAPI;
using namespace libnifalcon;


//#define DEBUG_PRINTOUTS

namespace NiFalconHapticsDeviceInternal {
  list< string > nifalcon_device_libs;
}

HAPIHapticsDevice::HapticsDeviceRegistration 
NiFalconHapticsDevice::device_registration
( "NiFalconHapticsDevice",
  &(newInstance< NiFalconHapticsDevice >),
  NiFalconHapticsDeviceInternal::nifalcon_device_libs );

/// Constructor. device_index is the index of falcon device
/// connected. Should not be larger than getNrConnectedFalconDevices() - 1.
NiFalconHapticsDevice::NiFalconHapticsDevice( unsigned int device_index ):
  com_thread( NULL ),
  com_func_cb_handle( -1 ),
  index( device_index ),
  device( new libnifalcon::FalconDevice ) {
  
  max_stiffness = 800;
}

NiFalconHapticsDevice::~NiFalconHapticsDevice() {
  delete device;
}

bool NiFalconHapticsDevice::initHapticsDevice( int _thread_frequency ) {
  
#ifdef NIFALCONAPI_LIBUSB
  device->setFalconComm<libnifalcon::FalconCommLibUSB>();
#else
#ifdef NIFALCONAPI_LIBFTD2XX
  device->setFalconComm<libnifalcon::FalconCommFTD2XX>();
#else
#ifdef NIFALCONAPI_LIBFTDI
  device->setFalconComm<libnifalcon::FalconCommLibFTDI>();
#endif
#endif
#endif
  
  device->setFalconFirmware<libnifalcon::FalconFirmwareNovintSDK>();
  device->setFalconKinematic<libnifalcon::FalconKinematicStamper>();
  device->setFalconGrip<libnifalcon::FalconGripFourButton>();
 
#ifdef DEBUG_PRINTOUTS
  // get number of connected falcons.
  unsigned int num_falcons;
#endif

  
#ifdef DEBUG_PRINTOUTS
  if(!device->getDeviceCount(num_falcons)) {
    H3DUtil::Console(LogLevel::Error) << "Cannot get device count" << std::endl;
  }

  H3DUtil::Console(LogLevel::Error) << "Falcons found: " << num_falcons << std::endl;

  std::cout << "Opening falcon " << index << std::endl;

#endif

  if(!device->open( index )) {
    stringstream s;
    s << "Cannot open falcon(index " << index << ") - Error: " 
      << device->getErrorCode() 
      << ". Make sure you have the device connected properly "
      << "and have the permissions to communicate over the USB "
      << "port. " << std::endl;
    setErrorMsg( s.str() );
    return false;
  }

  if(!device->isFirmwareLoaded()) {
    bool has_firmware = false;
    for(int i = 0; i < 10; ++i) {
      if(!device->getFalconFirmware()->
        loadFirmware(true, 
                     NOVINT_FALCON_NVENT_FIRMWARE_SIZE,
                     const_cast<uint8_t*>(NOVINT_FALCON_NVENT_FIRMWARE)))
        {
#ifdef DEBUG_PRINTOUTS
    std::cout << "Cannot load firmware" << std::endl;
#endif
        }
      else
        {
    has_firmware = true;
#ifdef DEBUG_PRINTOUTS
    std::cout <<"firmware loaded" << std::endl;
#endif
    break;
        }
    }

    if(!has_firmware) {
      setErrorMsg( "Could not load Falcon firmware(libnifalcon)." );
      return false;
    }
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
  
  hd->device->getFalconFirmware()->setHomingMode( true );
  hd->device->runIOLoop();
  if( hd->device->getFalconFirmware()->isHomed() ) {

    // get position
    boost::array<double,3> pos = hd->device->getPosition();
    Vec3 position( pos[0] , pos[1] , (pos[2] - .150f) );
    
    // get buttons
    boost::shared_ptr<FalconGrip> grip = hd->device->getFalconGrip();
    // It is safe to not check the number of digital outputs since
    // getDigitalInput in FalconGrip.h checks this for us.
    // Button input are grabbed this way in order to get them in the
    // same order as for FalconHapticsDevice (using novints api).
    HAPIInt32 button_status = // [ + N (logo) V ] -> [ logo V N + ]
      (grip->getDigitalInput(0)?0x08:0x00)|
      (grip->getDigitalInput(1)?0x04:0x00)|
      (grip->getDigitalInput(2)?0x01:0x00)|
      (grip->getDigitalInput(3)?0x02:0x00);

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
    hd->device->setForce(f);
 }
  
  return H3DUtil::PeriodicThread::CALLBACK_CONTINUE;
}

bool NiFalconHapticsDevice::setDeviceIndex( unsigned int i ) {
  if( getDeviceState() == HAPIHapticsDevice::UNINITIALIZED ) {
    index = i;
    return true;
  } else {
    return false;
  }
}

unsigned int NiFalconHapticsDevice::getNrConnectedFalconDevices() {
  libnifalcon::FalconDevice device;
#ifdef NIFALCONAPI_LIBUSB
  device.setFalconComm<libnifalcon::FalconCommLibUSB>();
#else
#ifdef NIFALCONAPI_LIBFTD2XX
  device.setFalconComm<libnifalcon::FalconCommFTD2XX>();
#else
#ifdef NIFALCONAPI_LIBFTDI
  device.setFalconComm<libnifalcon::FalconCommLibFTDI>();
#endif
#endif
#endif
  unsigned int num_falcons;
  if(!device.getDeviceCount(num_falcons)) {
    return 0;
  }
  return num_falcons;
}

#endif
