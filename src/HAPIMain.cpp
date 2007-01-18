#include <AnyHapticsDevice.h>
#include <HapticForceField.h>
#include <HapticLineConstraint.h>
#include <HapticGeometryConstraint.h>
#include <HapticViscosity.h>
#include <HapticShapeConstraint.h>
#include <HapticSphere.h>
#include <HapticTriangleSet.h>
#include <HapticTimeFunctionEffect.h>
#include <HapticPositionFunctionEffect.h>
#include <ParsedFunction.h>

using namespace HAPI;

int main(int argc, char* argv[]) {
  AnyHapticsDevice hd;

  HapticForceField *force_field = new HapticForceField( Matrix4(),
                                                        Vec3( 1, 0, 0 ),
                                                        false );

  //HapticLineConstraint *haptic_line_constraint =
  //  new HapticLineConstraint( Matrix4(),
  //                            false,
  //                            Vec3(0, 0, 0 ),
  //                            Vec3( 1000, 0, 0),
  //                            0.3 );

  vector< Bounds::Triangle > tris;
  tris.push_back( Bounds::Triangle( Vec3( 0, 0, 0 ),
                                    Vec3( 50, 0, 0 ),
                                    Vec3( 50, 50, 0 ) ) );
  tris.push_back( Bounds::Triangle( Vec3( 50, 50, 0 ),
                                    Vec3( 0, 50, 0 ),
                                    Vec3( 0, 0, 0 ) ) );
  tris.push_back( Bounds::Triangle( Vec3( -50, 50, 30 ),
                                    Vec3( -50, 0, -30 ),
                                    Vec3( 0, 0, 0 ) ) );
 /* HapticGeometryConstraint *haptic_geometry_constraint =
    new HapticGeometryConstraint( Matrix4(), false, tris, 0.3 );*/

  HapticViscosity * haptic_viscosity =
    new HapticViscosity( Matrix4(), 0.00089 / 1000,
                         0.0025 * 1000, 0.8, false );

  void *user_data = 0;
  HAPISurfaceObject * my_surface = new HAPISurfaceObject();
  HapticSphere *my_haptic_sphere =
    new HapticSphere( 50, true, user_data, my_surface, Matrix4() );
  HapticTriangleSet *my_haptic_triangle_set =
    new HapticTriangleSet( tris, user_data, my_surface, Matrix4() );
  HapticShapeConstraint *haptic_shape_constraint =
    new HapticShapeConstraint( Matrix4(), false, my_haptic_triangle_set, 0.3 );

  ParsedFunction * parsed_function = new ParsedFunction();
  parsed_function->setFunctionString( "x*0 + y + 0*z", "x,y,z" );
  ParsedFunction * parsed_function2 = new ParsedFunction();
  parsed_function2->setFunctionString( "0", "x,y,z" );
  HapticTimeFunctionEffect * haptic_time_function_effect =
    new HapticTimeFunctionEffect( Matrix4(), false, parsed_function,
                                  parsed_function2, parsed_function2, 0, 0, 0);

  HapticPositionFunctionEffect * haptic_spatial_function_effect =
    new HapticPositionFunctionEffect( Matrix4(), false, parsed_function,
                                     parsed_function2, parsed_function2 );

  if( hd.initDevice() != HAPIHapticsDevice::SUCCESS ) {
    cerr << hd.getLastErrorMsg() << endl;
    return 0;
  }
  hd.enableDevice();
  // take whatever force you want.
  //hd.addEffect( force_field );
  //hd.addEffect( haptic_line_constraint );
  //hd.addEffect( haptic_geometry_constraint );
  //hd.addEffect( haptic_viscosity );
  //hd.addEffect( haptic_shape_constraint );
  //hd.addEffect( haptic_time_function_effect );
  hd.addEffect( haptic_spatial_function_effect );
  
  while( true ) {
   // hd.renderHapticsOneStep();
  }
}
