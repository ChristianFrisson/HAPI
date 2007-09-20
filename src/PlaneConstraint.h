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
// TODO: 
// Base code that this code was built upon was contributed by .....
//
//
/// \file CollisionStructures.h
/// \brief Header file for CollisionStructures, 
///
//
//////////////////////////////////////////////////////////////////////////////
#ifndef __PLANECONSTRAINT_H__
#define __PLANECONSTRAINT_H__

#include <IntersectionInfo.h>
#include <HAPIHapticShape.h>

namespace HAPI {

  namespace Bounds {

    class GeometryPrimitive;

    class HAPI_API PlaneConstraint {

    public:
      PlaneConstraint( const Vec3 &p = Vec3(), 
                       const Vec3 &n = Vec3(1, 0, 0), 
                       const Vec3 &tc = Vec3(), 
                       HAPIHapticShape *shape = NULL,
                       GeometryPrimitive *_primitive = NULL ):
      point( p ), normal( n ), tex_coord( tc ), 
        haptic_shape( shape ), primitive( _primitive )  {}

      bool lineIntersect( const Vec3 &from, 
                          const Vec3 &to,    
                          Bounds::IntersectionInfo &result );
      Vec3 point, normal;
      Vec3 tex_coord;
      H3DUtil::AutoRef< HAPIHapticShape > haptic_shape;
      GeometryPrimitive * primitive;
    };
  }
  
  class Constraints {
  public:
	  Constraints( unsigned int size = 3000 ) {
      constraints = new PlaneConstraint[size];
	    //constraints = (PlaneConstraint *) malloc( sizeof( PlaneConstraint ) * size );
      min_allocated_constraints = size;
      allocated_constraints = size;
      nr_constraints = 0;
	  }
    
    ~Constraints() {
      delete [] constraints;
    }
    
    class iterator {
    public:
      const static unsigned int end_value = 1929294;
      
      inline iterator( unsigned int _index = end_value, 
                       unsigned int _size = 0, 
                       Constraints *o = NULL ):
        index( _index ), size( _size ), owner( o ) {
        
      }
      
      inline iterator& operator++(void) {
        if( index != end_value ) index++;
        if( index == size ) index = end_value;
        return *this;
      }
      
      inline iterator operator++(int) {                    
        iterator old_val(*this);  
        operator++();    
        return old_val;   
      }  

      inline iterator operator+(int i) {  
        if( index != end_value ) index += i;
        if( index >= size ) index = end_value;
        return *this;
      }  

      inline PlaneConstraint & operator*() {
		  return owner->constraints[index];
	  }
      inline bool operator==( const iterator &i ) {
	    return index == i.index; 
	  }
      inline bool operator!=( const iterator &i ) {
	    return index != i.index;
	  }

      unsigned int index, size;
      Constraints *owner;

    };


    inline PlaneConstraint & operator[]( int i ) {
	    return constraints[i]; 
	  }

    inline PlaneConstraint & operator[]( unsigned int i ) {
	    return constraints[i]; 
	  }

    inline iterator begin() { 
		if( !empty() ) return iterator( 0, nr_constraints, this );
		else return end();
	}
    inline iterator end() {
		return iterator( iterator::end_value, nr_constraints, this );
	}
    inline void clear() { 
      for( unsigned int i = 0; i < nr_constraints; i++ )
        constraints[i].haptic_shape.reset( NULL );
      nr_constraints = 0;

    }
    inline bool empty() { return nr_constraints == 0; }
    inline unsigned int size() { return nr_constraints; }


    // TODO: implement properly
    inline void insert( iterator pos, iterator s, iterator e ) {
      for( iterator i = s; i != e; i++ )
        push_back( *i );
    }
    
	inline void push_back( const PlaneConstraint &p ) {
	  constraints[nr_constraints] = p;
	  nr_constraints++;
    }

    inline PlaneConstraint & front() { return constraints[0];}
    inline PlaneConstraint & back() { return constraints[nr_constraints-1]; }

    unsigned int min_allocated_constraints;
    unsigned int allocated_constraints;
    unsigned int nr_constraints;
    PlaneConstraint *constraints;
  };

}
#endif
