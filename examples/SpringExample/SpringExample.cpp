#include <HapticSpring.h>
#include <AnyHapticsDevice.h>

using namespace HAPI;

int main(int argc, char* argv[]) {
  AnyHapticsDevice hd;

  if( hd.initDevice() != HAPIHapticsDevice::SUCCESS ) {
    cerr << hd.getLastErrorMsg() << endl;
    system("PAUSE");
    return 0;
  }
  hd.enableDevice();

  bool continue_trying = true;

  while( continue_trying ) {
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

    HapticSpring *spring_effect = new HapticSpring( Matrix4(),
      Vec3( x, y, z ),
      spring_constant,
      false );
    hd.addEffect( spring_effect );

    cout << "Type q and then ENTER to exit, any other input and then ";
    cout << "ENTER will allow you to a new point and a new springconstant"
      << endl;
    getline( cin, thePosition );
    if( thePosition == "q" ) {
      continue_trying = false;
    }
    hd.removeEffect( spring_effect );
  }
  hd.disableDevice();
  hd.releaseDevice();
}
