//////////////////////////////////////////////////////////////////////////////
//    Copyright 2004, SenseGraphics AB
//
//    This file is part of H3D API.
//
//    H3D API is free software; you can redistribute it and/or modify
//    it under the terms of the GNU General Public License as published by
//    the Free Software Foundation; either version 2 of the License, or
//    (at your option) any later version.
//
//    H3D API is distributed in the hope that it will be useful,
//    but WITHOUT ANY WARRANTY; without even the implied warranty of
//    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//    GNU General Public License for more details.
//
//    You should have received a copy of the GNU General Public License
//    along with H3D API; if not, write to the Free Software
//    Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
//
//    A commercial license is also available. Please contact us at 
//    www.sensegraphics.com for more information.
//
//
/// \file RuspiniRenderer.h
/// \brief Header file for RuspiniRenderer.
///
//
//////////////////////////////////////////////////////////////////////////////

#ifndef __RUSPINIRENDERER_H__
#define __RUSPINIRENDERER_H__

#include <HAPIHapticsRenderer.h>
#include <Threads.h>
#include <map>

namespace HAPI {

  /// \class RuspiniRenderer
  /// Base class for all haptic devices. 
  class HAPI_API RuspiniRenderer: public HAPIHapticsRenderer {
  public:

    virtual ~RuspiniRenderer() {}

    virtual HapticForceEffect::EffectOutput 
    renderHapticsOneStep( HapticForceEffect::EffectInput input,
                          const HapticShapeVector &shapes );

    inline const Vec3 &getProxyPosition() {
      return proxy_position;
    }

    inline HAPIFloat getProxyRadius() {
      return proxy_radius;
    }

    inline void setProxyRadius( HAPIFloat r ) {
      proxy_radius = r;
    }
    
 /*   struct lt {
      bool operator()(HAPIHapticShape *s1, HAPIHapticShape *s2) const {
        if( s1 == s2 ) {
          return s1->id < s2->id;
        } else {
          return s1 < s2;
        }
      }
    };*/

    typedef std::vector< pair< H3DUtil::AutoRef< HAPIHapticShape >, HAPISurfaceObject::ContactInfo> > Contacts; 

    inline Contacts getContacts() {
      contacts_lock.lock();
      Contacts c = contacts;
      contacts_lock.unlock();
      return c;
    }

  protected:
    void onOnePlaneContact( const PlaneConstraint &c, 
                            HAPISurfaceObject::ContactInfo &contact );

    void onTwoPlaneContact( const PlaneConstraint &p0,
                            const PlaneConstraint &p1,
                            HAPISurfaceObject::ContactInfo &contact );

    void onThreeOrMorePlaneContact(  vector< PlaneConstraint > &constraints,
                                     HAPISurfaceObject::ContactInfo &contact );
    
    HAPIFloat proxy_radius;
    Vec3 proxy_position;
    MutexLock contacts_lock;
    Contacts contacts;
    Contacts tmp_contacts;

  };
}

#endif
