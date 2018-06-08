#include <HAPI/AnyHapticsDevice.h>
#include <HAPI/HapticForceField.h>

using namespace HAPI;

int main(int argc, char* argv[]) {
  // The force to render
  Vec3 force_to_render = Vec3( 1, 0, 0 );

  // Create a new haptics device, using any device connected.
  auto_ptr< AnyHapticsDevice > device( new AnyHapticsDevice );

  // initialize the device
  if( device->initDevice() != HAPIHapticsDevice::SUCCESS ) {
    // initilization failed, print error message and quit
    cerr << device->getLastErrorMsg() << endl;
    return 1;
  }
  
  // enable the device(forces and positions will be updated)
  device->enableDevice();

  // add the force effect to render
  device->addEffect( new HapticForceField( force_to_render ) );
  
  // transfer the effect to the haptics loop.
  device->transferObjects();

  // wait for keyboard ENTER press, then finish program 
  string temp_string;
  getline( cin, temp_string );

  // release the device.
  device->releaseDevice();
  
  return 0;
}
