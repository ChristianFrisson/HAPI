//////////////////////////////////////////////////////////////////////////////
//    Copyright 2004-2019, SenseGraphics AB
//
//    This file is part of HAPI.
//
//    HAPI is free software; you can redistribute it and/or modify
//    it under the terms of the GNU General Public License as published by
//    the Free Software Foundation; either version 2 of the License, or
//    (at your option) any later version.
//
//    HAPI is distributed in the hope that it will be useful,
//    but WITHOUT ANY WARRANTY; without even the implied warranty of
//    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//    GNU General Public License for more details.
//
//    You should have received a copy of the GNU General Public License
//    along with HAPI; if not, write to the Free Software
//    Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
//
//    A commercial license is also available. Please contact us at 
//    www.sensegraphics.com for more information.
//
//
/// \file HAPIShapeRenderOptions.h
/// \brief Header file for HAPIShapeRenderOptions
///
//
//////////////////////////////////////////////////////////////////////////////
#ifndef __HAPISHAPERENDEROPTIONS_H__
#define __HAPISHAPERENDEROPTIONS_H__

#include <HAPI/HAPI.h>

namespace HAPI {

  /// \ingroup AbstractClasses
  /// \class HAPIShapeRenderOptions
  /// \brief Base class for haptic shapes render options.
  ///
  /// Haptic shapes render options are classes that
  /// can be set in order to give shape specific options to a 
  /// HAPIHapticsRenderer of how to render a certain shape.
  class HAPI_API HAPIShapeRenderOptions {
  public:
    virtual ~HAPIShapeRenderOptions() {}
  };
}

#endif
