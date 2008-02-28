#include <HAPI/HAPIForceEffect.h> 

namespace HAPI {
  /// This is a HAPIForceEffect that generates a constant force.
  class HAPI_API HapticForceField: public HAPIForceEffect {
  public:

    // Constructor.
    HapticForceField( const Vec3 &_force ):
      force( _force ) {}
    
    /// The force of the EffectOutput will be the force of the force field. 
    EffectOutput virtual calculateForces( const EffectInput &input ) {
      return EffectOutput( force );
    }
    
  protected:
    Vec3 force;
  };
}
