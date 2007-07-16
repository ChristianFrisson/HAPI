#include <AnyHapticsDevice.h>
#include <HapticSphere.h>
#include <FrictionSurface.h>
#include <GodObjectRenderer.h>

using namespace HAPI;

int main(int argc, char* argv[]) {
  AnyHapticsDevice hd;

  HAPISurfaceObject * my_surface = new FrictionSurface();
  HapticSphere *my_haptic_sphere =
    new HapticSphere( 50, 0, my_surface, Matrix4() );

  hd.setHapticsRenderer( new GodObjectRenderer() );
  if( hd.initDevice() != HAPIHapticsDevice::SUCCESS ) {
    cerr << hd.getLastErrorMsg() << endl;
    return 0;
  }
  hd.enableDevice();
  
  hd.addShape( my_haptic_sphere );

  hd.transferObjects();

  cout << "Provided you have a haptics device connected the proxy ";
  cout << "will now be inside a sphere." << endl << endl;
  string temp_string;
  cerr << "Press ENTER to exit" << endl;
  getline( cin, temp_string );
  hd.disableDevice();
  hd.releaseDevice();
}
