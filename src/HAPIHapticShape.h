//////////////////////////////////////////////////////////////////////////////
//    Copyright 2004-2007, SenseGraphics AB
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
#include <list>

namespace HAPI {


  /// \brief Base class for haptic shapes, i.e. geometrical objects that are
  /// rendered by letting their surfaces constrain the proxy. A HAPIHapticShape
  /// has a Surface object associated to it that defines the properties of the
  /// surface, e.g. stiffness and friction properties.
  class HAPI_API HAPIHapticShape: public HAPIHapticObject,
                                  public Collision::CollisionObject,
                                  public HAPIGLShape {
  public:


    /// Constructor.
    HAPIHapticShape( void *_userdata,
                     HAPISurfaceObject *_surface,
                     const Matrix4 & _transform,
                     void (*_clean_up_func)( void * ) = 0,
                     int _shape_id = -1,
                     Collision::FaceType _touchable_face = 
                     Collision::FRONT_AND_BACK  ):
                  Collision::CollisionObject( true ),
                  HAPIHapticObject( _transform ),
                  clean_up_func( _clean_up_func ),
                  surface( _surface ),
                  userdata( _userdata ),
                  shape_id( _shape_id ),
                  touchable_face( _touchable_face ) {}

    virtual ~HAPIHapticShape();

    virtual void closestPoint( const Vec3 &p, Vec3 &cp, Vec3 &n, Vec3 &tc ) {
      // todo: fix
    }

    virtual bool lineIntersect( const Vec3 &from, 
                                const Vec3 &to,
                                Collision::IntersectionInfo &result,
                                Collision::FaceType face ) { 
      return false;
    }

    virtual bool movingSphereIntersect( HAPIFloat radius,
                                        const Vec3 &from, 
                                        const Vec3 &to ) { 
      return false;
    }

    virtual void getConstraints( const Vec3 &point,
                                 Constraints &constraints,
                                 Collision::FaceType face = Collision::FRONT_AND_BACK,
                                 HAPIFloat radius = -1 );
    
    static int genShapeId();
    
    static void delShapeId( int id );

    /// The Surface object describing the properties of the surface.
    H3DUtil::AutoRef< HAPISurfaceObject > surface;

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
    Collision::FaceType touchable_face;

    static int current_max_id;
    static list< int > free_ids;

  protected:
    void (*clean_up_func)( void * );
  };
}

#endif
