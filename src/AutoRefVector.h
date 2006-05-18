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
/// \file AutoRefVector.h
/// Header file for AutoRefVector class.
///
//
//////////////////////////////////////////////////////////////////////////////
#ifndef __AUTOREFVECTOR_H__
#define __AUTOREFVECTOR_H__

#include <HAPI.h>
#include <vector>
#include <algorithm>

using namespace std;

namespace H3D {
  /// This class is similar to the AutoRef class in the vector elements
  /// are Node * or pointers to subclasses of Node. Reference counting 
  /// will be upheld on all nodes in the vector.
  /// 
  template< class NodeClass >
  class AutoRefVector : private vector<NodeClass*> {
  public:
    /// The type of the Node, NodeClass, stored in the vector.
    typedef typename vector<NodeClass*>::value_type value_type;
    /// Pointer to NodeClass.
    typedef typename vector<NodeClass*>::pointer pointer;
    /// Const reference to NodeClass.
    typedef typename vector<NodeClass*>::const_reference const_reference;
    /// An unsigned integral type.
    typedef typename vector<NodeClass*>::size_type size_type;
    /// A signed integral type.
    typedef typename vector<NodeClass*>::difference_type difference_type; 
	 /// Const iterator used to iterate through a vector.
    typedef typename vector<NodeClass*>::const_iterator const_iterator;
    /// Iterator used to iterate backwards through a vector.
    typedef typename vector<NodeClass*>::const_reverse_iterator 
    const_reverse_iterator;

    /// Creates an empty vector.
    inline AutoRefVector() {}

    /// Copy constructor from a vector class.
    inline AutoRefVector( const vector<NodeClass *> &v ) : 
      vector<NodeClass*>( v ) {
      refAll();
    }

    /// Copy constructor
    inline AutoRefVector( const AutoRefVector<NodeClass> &v ) : 
      vector<NodeClass*>( v ) {
      refAll();
    }

    /// Creates a vector with n elements.
    inline AutoRefVector( size_type n ):
      vector< NodeClass * >( n ) {}

    /// Destructor.
    inline virtual ~AutoRefVector() {
      clear();
    }

    /// Assignement operator.
    inline AutoRefVector<NodeClass> 
    &operator=( const AutoRefVector<NodeClass> &v ) {
      if( this != &v ) {
        unrefAll();      
        vector<NodeClass*>::operator=( v );
        refAll();
      }
      return *this;
    }

    /// Assignement operator.
    inline AutoRefVector<NodeClass> &operator=( 
                                               const vector<NodeClass *> &v ) {
      // temporarily add an extra reference to the nodes in v so
      // they are not accidentally removed in unrefAll()
      for( typename vector< NodeClass * >::const_iterator i = v.begin();
           i != v.end();
           i++ ) 
        (*i)->ref();
      unrefAll();
      vector<NodeClass*>::operator=( v );
      refAll();

      // remove the temporary references.
      for( typename vector< NodeClass * >::const_iterator i = v.begin();
           i != v.end();
           i++ ) 
        (*i)->unref();
      return *this;
    }

    /// Returns a const_iterator pointing to the beginning of the vector.
    inline const_iterator begin() const { 
      return vector<NodeClass*>::begin();
    }
      
    /// Returns a const_iterator pointing to the end of the vector.
    inline const_iterator end() const { return vector<NodeClass*>::end(); }

        
    /// Returns a const_reverse_iterator pointing to the beginning of the
    /// reversed vector.
    inline const_reverse_iterator rbegin() const { 
      return vector<NodeClass*>::rbegin();
    }
      
    /// Returns a const_reverse_iterator pointing to the end of the reversed 
    /// vector.
    inline const_reverse_iterator rend() const { 
      return vector<NodeClass*>::rend(); 
    }

    /// Returns the size of the vector.
    inline size_type size() const { 
      return vector<NodeClass*>::size(); 
    }

    /// Returns the largest possible size of the vector.
    inline size_type max_size() const {
      return vector<NodeClass*>::max_size();
    }
        
    /// Number of elements for which memory has been allocated. capacity() 
    /// is always greater than or equal to size().
    inline size_type capacity() const { 
      return vector<NodeClass*>::capacity(); 
    }
        
    /// Swaps the contents of two vectors.
    inline void swap( AutoRefVector<NodeClass> &x ) {
      vector<NodeClass*>::swap( x );
    }

    /// Swaps the contents of two vectors.
    inline void swap( vector<NodeClass*> &x ) {
      unrefAll();
      vector<NodeClass*>::swap( x );
      refAll();
    }
        
    /// A request for allocation of additional memory. If n is less than
    /// or equal to capacity(), this call has no effect. 
    /// Otherwise, it is a request for allocation of additional memory. 
    /// If the request is successful, then capacity() is greater than or 
    /// equal to n; otherwise, capacity() is unchanged. In either case, 
    /// size() is unchanged.
    /// 
    inline void reserve( size_t s ) { vector<NodeClass*>::reserve( s ); }

    /// Inserts or erases elements at the end such that the size becomes n.
    inline virtual void resize( size_t n, NodeClass * t = NULL ) {
      if( size() > n ) {
        for( unsigned int i = size() - 1; i >= n; i-- )
          unref( vector<NodeClass*>::operator[]( i ) );
      }
      vector<NodeClass*>::resize( n, t );
    }

    /// true if the vector's size is 0.
    inline bool empty() const { return vector<NodeClass*>::empty(); }
    /// Returns the n'th element. We return a const_reference so that
    /// the values of the vector only can be changed using member 
    /// functions. To change the value of a specific index use
    /// the set( index, value ) function.
    inline const_reference operator[](size_type n) const {
      return vector<NodeClass*>::operator[]( n );
    }
    /// Set value at index i to v.
    inline void set( size_type i, const value_type &v ) {
      vector<NodeClass*>::operator[]( i ) = v;
    }
    /// Returns the first element.
    inline const_reference front() const { return vector<NodeClass*>::front(); }

    /// Returns the last element.
    inline const_reference back() const { return vector<NodeClass*>::back(); }

    /// Inserts a new element at the end.
    inline void push_back( const value_type &x ) {
      ref( x );
      vector< NodeClass * >::push_back( x );
    }

    /// Removed the last element.
    void pop_back() {
      unref( back() );
      vector< NodeClass * >::pop_back();
    }
        
    /// Erases all of the elements.
    inline void clear() {
      unrefAll();
      vector<NodeClass*>::clear();
    }

    /// Erase the first element equal to a.
    inline virtual void erase( NodeClass *a ) {
      typename vector<NodeClass * >::iterator i = 
        std::find( vector<NodeClass*>::begin(), 
                   vector<NodeClass*>::end(), 
                   a );
      if( i != end() ) {
        unref( *i );
        vector<NodeClass*>::erase( i );
      } 
    }

  protected:
    /// Virtual function that is called when a Node is added to 
    /// the vector.
    inline virtual void ref( NodeClass *n ) const {
      if( n ) {
        n->ref();
      }
    }

    /// Virtual function that is called when a Node is removed to 
    /// the vector.
    inline virtual void unref( NodeClass *n ) const {
      if( n ) {
        n->unref();
      }
    }

    /// Call ref () on all values in the vector.
    inline void refAll() const {
      for( const_iterator i = vector<NodeClass*>::begin(); 
           i != vector<NodeClass*>::end(); ++i ) 
        ref( *i );
    }

    /// Call unref () on all values in the vector.
    inline void unrefAll() const {
      for( const_iterator i = vector<NodeClass*>::begin(); 
           i != vector<NodeClass*>::end(); ++i ) 
        unref( *i );
    }
  };
}
    
#endif
