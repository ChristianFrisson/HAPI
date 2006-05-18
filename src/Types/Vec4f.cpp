#include "Vec4f.h"
#include "Vec4d.h"

H3D::ArithmeticTypes::Vec4f::Vec4f( const Vec4d &v ): x( (H3DFloat)v.x ), 
                                                      y( (H3DFloat)v.y ), 
                                                      z( (H3DFloat)v.z ),
                                                      w( (H3DFloat)v.w ){}
