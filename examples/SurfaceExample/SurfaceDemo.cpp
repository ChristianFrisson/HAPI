#include <HAPI/AnyHapticsDevice.h>
#include <HAPI/HapticPrimitive.h>
#include <HAPI/FrictionSurface.h>
#include <HAPI/GodObjectRenderer.h>

using namespace HAPI;

int main(int argc, char* argv[]) {
  AnyHapticsDevice hd;

  HAPISurfaceObject * my_surface = new FrictionSurface();
  HapticPrimitive *my_haptic_sphere =
	  new HapticPrimitive( new Collision::Sphere( Vec3( 0, 0, 0 ), 50 ), my_surface );

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
