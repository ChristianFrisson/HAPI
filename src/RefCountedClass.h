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
//
//
//////////////////////////////////////////////////////////////////////////////
#ifndef __REFCOUNTEDCLASS_H__
#define __REFCOUNTEDCLASS_H__

#include <HApi.h>
#include "Console.h"
#include <string>
#include <iostream>
using namespace std;

namespace H3D {

  class HAPI_API RefCountedClass {
  public:

    /// Constructor.
    RefCountedClass( ):
      ref_count( 0 ),
      name( "" ),
      type_name( "RefCountedClass" ),
      is_initialized( false ),
      manual_initialize( false ){}


    /// Destructor.
    virtual ~RefCountedClass() {
#ifdef REF_COUNT_DEBUG
      Console(1) << "~RefCountedClass: " << this << endl;
#endif
    }

    /// Initialize is called once upon the first reference of the 
    /// RefCountedClass.
    virtual void initialize() {
      is_initialized = true;
    }

    /// Increase the reference count for this instance.
    inline void ref() { 
      ref_count++;
#ifdef REF_COUNT_DEBUG
      Console(1) << "Ref " << getName() << " " << this << ": " 
                 << ref_count << endl;
#endif
      if( !manual_initialize && ref_count == 1 ) {
        initialize();
      }
    }

    /// If true, the initialize() function will not be called automatically
    /// on first reference of the instance but must be called manually
    /// by the creator of the instance.
    inline void setManualInitialize( bool b ) {
      manual_initialize = b;
    }

    /// Returns if the RefCountedClass has to be manually initialized or not.
    inline bool getManualInitialize() {
      return manual_initialize;
    }

    /// Decrease the reference count for this instance. If the reference
    /// count reaches 0 it is deleted.
    inline void unref() {
      ref_count--;
#ifdef REF_COUNT_DEBUG
      Console(1) << "Unref " << getName() << " " << this << ": " 
                 << ref_count << endl;
#endif
      if( ref_count == 0 ) {
        delete this;
      }
    }

    /// Get the name of the node.
    inline string getName() { 
      if( name == "" )
        return string( "Unnamed " ) + getTypeName();
      else 
        return name; 
    }

    /// Set the name of the node.
    inline void setName( const string &_name ) { 
      name = _name;
    }
    
    /// Returns true if this node has been given a name.
    inline bool hasName() { return name != ""; }

    /// Get the name of this Node type. E.g. if the Node is an IndexedFaceSet
    /// it should return "IndexedFaceSet"
    inline string getTypeName() {
      return type_name;
    }

    /// Returns true if the initialize() function has been called.
    inline bool isInitialized() {
      return is_initialized;
    }

  protected:
    /// The number of references to this instance.
    unsigned int ref_count; 

    /// The name
    string name;

    /// String version of the name of the type.
    string type_name;

    /// true if initialize() function has been called.
    bool is_initialized;

    /// If true, the initialize() function will not be called automatically
    /// on first reference of the instance but must be called manually
    /// by the creator of the instance.
    bool manual_initialize;
  };
    
}

#endif
