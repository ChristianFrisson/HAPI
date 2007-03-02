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
/// \file HAPIFunctionObject.h
/// \brief Header file for HAPIFunctionObject
///
//
//////////////////////////////////////////////////////////////////////////////
#ifndef __HAPIFUNCTIONOBJECT_H__
#define __HAPIFUNCTIONOBJECT_H__

#include <HAPI.h>
#include <HAPITypes.h>

namespace HAPI {
  /// \class HAPIFunctionObject
  /// \brief Base class for nodes evaluating function.
  /// The function is a function R^n -> R.
  class HAPI_API HAPIFunctionObject {
  public:
    /// Virtual destructor
	virtual ~HAPIFunctionObject() {}
  
    /// Evaluate the function. 
    /// input points to the input values to the function.
    virtual HAPIFloat evaluate( HAPIFloat *input ) = 0; 
  };
}

#endif
