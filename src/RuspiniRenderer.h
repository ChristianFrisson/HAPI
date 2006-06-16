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
/// \file RuspiniRenderer.h
/// \brief Header file for RuspiniRenderer.
///
//
//////////////////////////////////////////////////////////////////////////////

#ifndef __RUSPINIRENDERER_H__
#define __RUSPINIRENDERER_H__

#include <HAPIHapticsRenderer.h>

namespace H3D {

  /// \class RuspiniRenderer
  /// Base class for all haptic devices. 
  class HAPI_API RuspiniRenderer: public HAPIHapticsRenderer {
  public:

    virtual ~RuspiniRenderer() {}

    virtual HapticForceEffect::EffectOutput 
    renderHapticsOneStep( HapticForceEffect::EffectInput input,
                          const HapticShapeVector &shapes );

  protected:
    H3DDouble proxy_radius;
    Vec3d proxy_position;
  };
}

#endif
