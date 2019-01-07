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
/// \file SurfaceDemo.cpp
/// \brief CPP file which provides example code which lets the user feel the
/// surface of an object. No graphics is needed for this.
///
//
//////////////////////////////////////////////////////////////////////////////

// HAPI includes
#include <HAPI/AnyHapticsDevice.h>
#include <HAPI/HapticPrimitive.h>
#include <HAPI/FrictionSurface.h>
#include <HAPI/GodObjectRenderer.h>

using namespace HAPI;
using namespace std;

int main(int argc, char* argv[]) {
  // Get a connected device.
  AnyHapticsDevice hd;

  // The haptics renderer to use.
  hd.setHapticsRenderer( new GodObjectRenderer() );

  // Init the device.
  if( hd.initDevice() != HAPIHapticsDevice::SUCCESS ) {
    cerr << hd.getLastErrorMsg() << endl;
    return 0;
  }
  // Enable the device
  hd.enableDevice();
  
  // Creating a default surface.
  HAPISurfaceObject * my_surface = new FrictionSurface();
  // Creating a sphere with radius 0.05 and center in (0, 0, 0). Units in m.
  HapticPrimitive *my_haptic_sphere =  new HapticPrimitive(
    new Collision::Sphere( Vec3( 0, 0, 0 ), 0.05 ),
    my_surface, Collision::FRONT );

  // Add the shape to be rendered on the device.
  hd.addShape( my_haptic_sphere );

  // Transfer objects (shapes) to the haptics loop.
  hd.transferObjects();

  // Wait for user input.
  cout << "Provided you have a haptics device connected you should ";
  cout << "find a sphere in the middle of the device workspace."
       <<endl << endl;
  string temp_string;
  cerr << "Press ENTER to exit" << endl;
  getline( cin, temp_string );

  // Disable device.
  hd.disableDevice();

  // Release device.
  hd.releaseDevice();

  // No explicit deletion of pointers are neccessary in this example since all
  // created objects gets reference counted when sent to the haptics device.
  // Hence when the variable hd is destroyed ( program exit ) the other objects
  // will be deleted.
}
