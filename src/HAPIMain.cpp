#include <PhantomHapticsDevice.h>
#include <HapticForceField.h>

using namespace H3D;

int main(int argc, char* argv[]) {
  PhantomHapticsDevice hd;
  HapticForceField *force_field = new HapticForceField( Matrix4d(),
                                                        Vec3f( 1, 0, 0 ),
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
