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
/// \file HAPIHapticShape.h
/// \brief Header file for HAPIHapticShape
///
//
//////////////////////////////////////////////////////////////////////////////
#ifndef __HAPIHAPTICSHAPE_H__
#define __HAPIHAPTICSHAPE_H__

// HAPI includes
#include <HAPIHapticObject.h>
#include <HAPIGLShape.h>
#include <HAPISurfaceObject.h>
#include <HAPIShapeRenderOptions.h>
#include <RefCountedClass.h>
#include <CollisionObjects.h>

// H3D Util includes
#include <AutoPtrVector.h>

// stl includes
#include <vector>
#include <algorithm>

namespace HAPI {


  /// Base class for haptic shapes, i.e. geometrical objects that are rendered
  /// by letting their surfaces constrain the proxy. A HAPIHapticShape has a Surface
  /// object associated to it that defines the properties of the surface, e.g.
  /// stiffness and friction properties.
  class HAPI_API HAPIHapticShape: public HAPIHapticObject,
                                  public Bounds::CollisionObject,
                                  public HAPIGLShape {
  public:
    /// Constructor.
    HAPIHapticShape( void *_userdata,
                     HAPISurfaceObject *_surface,
                     const Matrix4 & _transform,
                     int _shape_id = -1 ):
      HAPIHapticObject( _transform ),
      surface( _surface ),
      userdata( _userdata ),
      shape_id( _shape_id ) {
      
    }

    virtual Vec3 closestPoint( const Vec3 &p ) {
      return Vec3();
    }

    virtual bool lineIntersect( const Vec3 &from, 
                                const Vec3 &to,
                                Bounds::IntersectionInfo &result ) { 
      return false;
    }

    static int genShapeId() {
      return 0;
    }
    
    static void delShapeId() { }

    /// The Surface object describing the properties of the surface.
    HAPISurfaceObject *surface;

    inline void addRenderOption( HAPIShapeRenderOptions *o ) {
      options.push_back( o );
    }

    inline void removeRenderOption( HAPIShapeRenderOptions *o ) {
      H3DUtil::AutoPtrVector< HAPIShapeRenderOptions >::iterator i =
        std::find( options.begin(), options.end(), o );
      if( i != options.end() ) {
        options.erase( i );
      }
    }

    inline const H3DUtil::AutoPtrVector< HAPIShapeRenderOptions > &getRenderOptions() {
      return options;
    }

    template< class T > 
    void getRenderOption( T * &option ) {
      for( H3DUtil::AutoPtrVector< HAPIShapeRenderOptions >::const_iterator i =
             options.begin();
           i != options.end(); i++ ) {
        T *o = dynamic_cast< T * >( *i );
        if( o ) {
          option = o;
          return;
        }
      }
      option = NULL;
    } 

    H3DUtil::AutoPtrVector< HAPIShapeRenderOptions > options;

    void *userdata;

    int shape_id;
      
  };
}

#endif
