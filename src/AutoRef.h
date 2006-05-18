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
/// \file AutoRef.h
/// Header file for AutoRef class.
///
//
//////////////////////////////////////////////////////////////////////////////
#ifndef __AUTOREF_H__
#define __AUTOREF_H__

namespace H3D {
  /// The AutoRef class is similar to the auto_ptr class, but it requires
  /// that the pointer to Node or a subclass of Node. It will keep a 
  /// reference to the Node pointer it encapsulates, i.e. ref() will be
  /// called. When destructed the AutoRef does not delete the pointer, 
  /// but instead unreferences the Node (if this causes the reference count
  /// for the Node to go down to 0 then it will be deleted though).
  ///
    
  template < class NodeType > 
  class AutoRef {
  public: 
    /// The type of the encapsulated Node.
    typedef NodeType element_type; 
        
    /// Constructor.   
    explicit AutoRef (NodeType* n = 0) throw() :
      node_ptr( n ) {
      ref( node_ptr );
    }

    /// Copy constructor.
    AutoRef(const AutoRef<NodeType>&ar) throw () :
      node_ptr( NULL ){
      reset( ar.get() );
    }
            
    /// Copy constructor from other type of AutoRef.
    template <class Y> 
    AutoRef(const AutoRef<Y>&ar) throw() :
      node_ptr( NULL ){
      reset( ar.get() );
    }
        
    /// Assignment operator.
    AutoRef<NodeType>& operator=(const AutoRef<NodeType>&ar) throw() {
      reset( ar.node_ptr );
      return *this;
    }

    /// Assignment operator for other type of AutoRef.
    template <class Y> 
    AutoRef<NodeType>& operator= (const AutoRef<Y>& ar) throw() {
      reset( ar.node_ptr );
      return *this; 
    } 
    virtual ~AutoRef() throw() {
      unref( node_ptr );
    } 
        
    /// Returns what the encapsulated Node * points to.  
    NodeType& operator* () const throw() {
      return *node_ptr;
    } 

    /// Returns the encapsulated Node pointer.
    NodeType* operator-> () const throw() {
      return node_ptr;
    } 

    /// Returns the encapsulated Node pointer.
    NodeType* get () const throw() {
      return node_ptr;
    }

    /// Change the Node pointer that is encapsulated. Will cause an
    /// unref on the current Node * and a ref on the new.
    /// \param p The new Node pointer to encapsulate.
    void reset(NodeType* p = 0) throw() {
      if( p != node_ptr ) {
        if ( node_ptr ) unref( node_ptr );
        node_ptr = p;
        if ( node_ptr ) ref( node_ptr );
      }
    }

  protected:
    /// This function is called when a Node * is to be held by the AutoRef.
    /// It increments the reference counter of the Node by calling the 
    /// ref() function. Subclasses can override this function in order to 
    /// get specialized behaviour.
    /// \param n The node that is to be held by the AutoRef
    ///
    inline virtual void ref( NodeType *n ) {
      if( n )
        n->ref();
    }

    /// This function is called when a Node * is released by the AutoRef. 
    /// It decrements the reference counter of the Node by calling the 
    /// unref() function. Subclasses can override this function in order to 
    /// get specialized behaviour.
    /// \param n The node being released by the AutoRef.
    /// 
    inline virtual void unref( NodeType *n ) {
      if( n )
        n->unref();
    }
        
    NodeType *node_ptr;

  };
}

#endif
