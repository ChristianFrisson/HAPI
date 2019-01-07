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
/// \file HAPIProxyBasedRenderer.h
/// \brief Header file for HAPIProxyBasedRenderer.
///
//
//////////////////////////////////////////////////////////////////////////////

#ifndef __HAPIPROXYBASEDRENDERER_H__
#define __HAPIPROXYBASEDRENDERER_H__

#include <HAPI/HAPIHapticsRenderer.h>

namespace HAPI {

  // forward declaration
  class HAPIProxyBasedDevice;

  /// \ingroup AbstractClasses
  /// \class HAPIProxyBasedRenderer
  /// \brief Base class for all proxy based haptic renderers in HAPI.
  ///
  /// A proxy is a virtual representation of the haptics device position that
  /// always stays outside any surface. Forces are then generated depending on the 
  /// difference between the proxy position and the haptics device position.
  class HAPI_API HAPIProxyBasedRenderer: public HAPIHapticsRenderer {
  public:
    /// Get the current proxy position.
    virtual Vec3 getProxyPosition() = 0;
  };
}

#endif
