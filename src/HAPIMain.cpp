#include <AnyHapticsDevice.h>
#include <HapticForceField.h>

using namespace HAPI;

int main(int argc, char* argv[]) {
  AnyHapticsDevice hd;
  HapticForceField *force_field = new HapticForceField( Matrix4(),
                                                        Vec3( 1, 0, 0 ),
                                                        false );
  if( hd.initDevice() != HAPIHapticsDevice::SUCCESS ) {
    cerr << hd.getLastErrorMsg() << endl;
    return 0;
  }
  hd.enableDevice();
  hd.addEffect( force_field );
  
  while( true ) {
   // hd.renderHapticsOneStep();
  }
}
