#include <HaptikDevice.h>
#include <HapticForceField.h>

using namespace H3D;

int main(int argc, char* argv[]) {
  HaptikDevice hd;
  HapticForceField *force_field = new HapticForceField( Matrix4f(),
                                                        Vec3f( 1, 0, 0 ),
                                                        false );
  hd.initDevice();
  hd.enableDevice();
  hd.addEffect( force_field );
  
  while( true ) {
    hd.renderHapticsOneStep();
  }
}
