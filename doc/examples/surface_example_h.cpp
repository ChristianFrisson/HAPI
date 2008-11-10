
// include the surface base class
#include <HAPI/HAPISurfaceObject.h>

namespace HAPI {

  // This class implements a surface without any friction and with
  // a linear spring interaction force pulling the probe back
  // towards the proxy. The force is defined as 
  // F = stiffness * (proxy - probe)
  class HAPI_API SmoothSurface : public HAPISurfaceObject {
  public:

    // Constructor. Takes the spring constant.
    SmoothSurface( HAPIFloat _stiffness = 350 ):
      stiffness( _stiffness ) {}

    // Moves the proxy along the surface in the direction of the
    // probe. 
    virtual void getProxyMovement( ContactInfo &contact_info ) {
      // get the position in the local contact coordinate system.
      Vec3 local_probe = contact_info.localProbePosition();

      // set the proxy movement to move to the projection of the probe
      // onto the local xz-plane.
      contact_info.setLocalProxyMovement( Vec2( local_probe.x , 
                                                local_probe.z ) );
    }

    // Calculate a force from the probe towards the proxy. 
    virtual void getForces( ContactInfo &contact_info ) {
      // origin is proxy position after proxy movement. probe_to_origin
      // is the vector from the probe to the proxy.
      Vec3 probe_to_origin = 
        contact_info.globalOrigin() - contact_info.globalProbePosition();

      // set the spring force
      contact_info.setGlobalForce(  stiffness*probe_to_origin );
    }

    // The stiffness of the surface.
    HAPIFloat stiffness;

  };
}

