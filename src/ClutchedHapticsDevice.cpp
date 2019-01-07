//////////////////////////////////////////////////////////////////////////////
//    Copyright 2011-2019, SenseGraphics AB
//
//    Any use, or distribution, of this file without permission from the
//    copyright holders is strictly prohibited. Please contact SenseGraphics,
//    www.sensegraphics.com, for more information.
//
//
/// \file ClutchedHapticsDevice.cpp
/// \brief Cpp file for ClutchedHapticsDevice.
///
//
//////////////////////////////////////////////////////////////////////////////

#include <HAPI/ClutchedHapticsDevice.h>

using namespace std;
using namespace HAPI;

bool ClutchedHapticsDevice::initHapticsDevice( int _thread_frequency ) {

  if ( hd ) {

    if( hd->initHapticsDevice( _thread_frequency ) ) {
      hd->device_state = HAPIHapticsDevice::INITIALIZED;
      setup_haptic_rendering_callback = hd->setup_haptic_rendering_callback;
      if( !setup_haptic_rendering_callback ) {
        hd->haptic_rendering_callback_data = this;
      }
      if( hd->thread )
        thread = hd->thread;
      max_stiffness = hd->getMaxStiffness();
      
      return true;
    } else {
      stringstream s;
      s << "Could not init clutched haptics device. Wrapped device "
        << "did not initialise properly. " << ends;
      setErrorMsg( s.str() );
      return false;
    }

  } else {
    stringstream s;
    s << "Could not init clutched haptics device. No wrapped device "
      << "was specified before initialization. " << ends;
    setErrorMsg( s.str() );
    return false;
  }
}

void ClutchedHapticsDevice::enableClutch ( bool enable ) {
  clutchLock.lock();

  DeviceValues dv= getUnclutchedDeviceValues();

  if ( enable ) {
    startClutchOrientation= dv.orientation*-clutchOrientationOffset;
    startClutchPosition= dv.position+clutchPositionOffset;
    clutchEnabled= true;
  } else {
    clutchOrientationOffset= -startClutchOrientation*dv.orientation;
    clutchPositionOffset= startClutchPosition-dv.position;
    clutchEnabled= false;
  }

  clutchLock.unlock();
}