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
/// \file AnyHapticsDevice.cpp
/// \brief Cpp file for AnyHapticsDevice.
///
//
//////////////////////////////////////////////////////////////////////////////

#include <HAPI/AnyHapticsDevice.h>

#include <sstream>
#include <H3DUtil/DynamicLibrary.h>

using namespace HAPI;
using namespace std;

HAPIHapticsDevice::HapticsDeviceRegistration 
AnyHapticsDevice::device_registration(
                            "Any",
                            &(newInstance< AnyHapticsDevice >),
                            std::list< std::string >()
                            );

bool AnyHapticsDevice::initHapticsDevice( int _thread_frequency ) {
  hd.reset( NULL );
  for( std::list< HapticsDeviceRegistration >::iterator i = 
         registered_devices->begin(); 
       i != registered_devices->end(); ++i ) {
    if( (*i).name != "Any" ) {
#ifdef H3D_WINDOWS
      /// need to go through list of libs to see if it is even
      /// possible to try to initialize the device.
      bool all_libs_ok = true;
      for( list< string >::iterator j = (*i).libs_to_support.begin();
           j != (*i).libs_to_support.end();
           ++j ) {
        if( H3DUtil::DynamicLibrary::load( *j ) == NULL ) {
          all_libs_ok = false;
          break;
        }
      }
      if( all_libs_ok ) {
#endif // H3D_WINDOWS
      HAPIHapticsDevice *device = ((*i).create_func)();
      if( device ) { // Device should never be NULL here, but who knows.
        if( device->initHapticsDevice( _thread_frequency ) ) {
          hd.reset( device );
          hd->device_state = HAPIHapticsDevice::INITIALIZED;
          setup_haptic_rendering_callback = hd->setup_haptic_rendering_callback;
          if( !setup_haptic_rendering_callback ) {
            hd->haptic_rendering_callback_data = this;
          }
          if( hd->thread )
            thread = hd->thread;
          max_stiffness = hd->getMaxStiffness();
          break;
        } else {
          delete device;
        }
      }
#ifdef H3D_WINDOWS
      }
#endif // H3D_WINDOWS
    }
  }

  if( !hd.get() ) {
    stringstream s;
    s << "Could not init any haptics device. Make sure one is "
      << "connected properly. " << ends;
    setErrorMsg( s.str() );
    return false;
  } else {
    hd->setErrorHandler( error_handler.get() );
    return true;
  }
}
