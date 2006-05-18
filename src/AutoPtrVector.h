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
/// \file AutoPtrVector.h
/// Header file for AutoPtrVector class.
///
//
//////////////////////////////////////////////////////////////////////////////
#ifndef __AUTOPTRVECTOR_H__
#define __AUTOPTRVECTOR_H__

#include <HAPI.h>
#include <vector>

using namespace std;

namespace H3D {
  /// This class is similar to the auto_ptr class in the vector elements
  /// are Node * or pointers to subclasses of Node. Reference counting 
  /// will be upheld on all nodes in the vector.
  /// 
  template< class PtrType >
  class AutoPtrVector : public vector<PtrType*> {
  public:
    /// Creates an empty vector.
    inline AutoPtrVector() {}

    /// Copy constructor from a vector class.
    inline AutoPtrVector( const vector<PtrType> &v ) : 
      vector<PtrType*>( v ) {
    }

    /// Copy constructor
    inline AutoPtrVector( const AutoPtrVector<PtrType> &v ) : 
      vector<PtrType*>( v ) {
    }

    /// Creates a vector with n elements.
    inline AutoPtrVector( typename vector<PtrType*>::size_type n ):
      vector< PtrType * >( n ) {}

    /// Destructor.
    inline virtual ~AutoPtrVector() {
      clear();
    }

    /// Assignement operator.
    inline AutoPtrVector<PtrType> 
    &operator=( const AutoPtrVector<PtrType> &v ) {
      vector<PtrType*>::operator=( v );
      return *this;
    }

    /// Removed the last element.
    void pop_back() {
      delete this->back();
      vector< PtrType * >::pop_back();
    }
        
    /// Erases the element at position pos.
    inline void erase( typename vector<PtrType*>::const_iterator pos ) { 
      delete *pos;
      vector<PtrType*>::erase( pos );
    }

    /// Erases the range [first, last)
    inline void erase( typename vector<PtrType*>::const_iterator first, 
                       typename vector<PtrType*>::const_iterator last ) {
      for( typename vector<PtrType*>::const_iterator i = first; i != last; ++i ) unref( *i );
      vector<PtrType*>::erase( first, last );
    }

    /// Erases all of the elements.
    inline void clear() {
      for( typename vector<PtrType*>::iterator i = this->begin(); 
           i != this->end(); i++ ) 
        delete (*i);
      vector<PtrType*>::clear();
    }
  };
}
    
#endif
