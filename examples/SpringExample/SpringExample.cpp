//////////////////////////////////////////////////////////////////////////////
//    Copyright 2004-2007, SenseGraphics AB
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
/// \file SpringExample.cpp
/// \brief CPP file which provides example code to show how to use force
/// effects in HAPI.
///
//
//////////////////////////////////////////////////////////////////////////////

// HAPI includes
#include <HAPI/HapticSpring.h>
#include <HAPI/AnyHapticsDevice.h>

using namespace HAPI;

int main(int argc, char* argv[]) {
  // Get a connected device.
  AnyHapticsDevice hd;

  // Init the device.
  if( hd.initDevice() != HAPIHapticsDevice::SUCCESS ) {
    cerr << hd.getLastErrorMsg() << endl;
    system("PAUSE");
    return 0;
  }
  // Enable the device
  hd.enableDevice();

  // used to loop the program until user wants to exit.
  bool continue_trying = true;

  while( continue_trying ) {
    // Get info from user.
    cout << "Enter a position as x y z (in mm) then press ENTER" << endl;
    string thePosition;
    getline( cin, thePosition );

    char * pEnd;
    HAPIFloat x, y, z;
    x = strtod( thePosition.c_str(), &pEnd );
    y = strtod( pEnd, &pEnd );
    z = strtod( pEnd, NULL );

    cout << "Enter the value for a spring constant then press ENTER." << endl;
    cout << "A good value is 0.1 since HAPI uses mm." << endl;
    cout << "Keep a firm grip of the device before pressing ENTER." << endl;
    getline( cin, thePosition );
    HAPIFloat spring_constant = strtod( thePosition.c_str(), NULL );

    stringstream stm_x, stm_y, stm_z, stm_constant;
    stm_x << x;
    stm_y << y;
    stm_z << z;
    stm_constant << spring_constant;
    cout << "The haptic device will be pulled towards position ";
    cout << stm_x.str() << " ";
    cout << stm_y.str() << " " << stm_z.str() << endl;
    cout << "The spring constant used is: " << stm_constant.str() << endl;
    cout << "Any faulty input number will be replaced by 0" << endl << endl;

    // The spring effect with a position and spring_constant input by the user.
    HapticSpring *spring_effect = new HapticSpring( Vec3( x, y, z ),
                                                    spring_constant );
    // Add the effect to the haptics device. This sends the effect to the
    // haptics loop and from now on it will be used to send forces to the
    // device.
    hd.addEffect( spring_effect );

    cout << "Type q and then ENTER to exit, any other input and then ";
    cout << "ENTER will allow you to a new point and a new springconstant"
      << endl;
    getline( cin, thePosition );
    if( thePosition == "q" ) {
      continue_trying = false;
    }
    // Remove the effect from the haptics device. The haptic effect will stop
    // affecting the haptics device since it is removed from the haptics loop.
    hd.removeEffect( spring_effect );
  }
  // Disable device.
  hd.disableDevice();
  // Release device.
  hd.releaseDevice();
}
