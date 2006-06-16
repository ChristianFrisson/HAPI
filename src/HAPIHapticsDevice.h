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
/// \file HAPIHapticsDevice.h
/// \brief Header file for HAPIHapticsDevice.
///
//
//////////////////////////////////////////////////////////////////////////////

#ifndef __HAPIHAPTICSDEVICE_H__
#define __HAPIHAPTICSDEVICE_H__

#include <HApi.h>
#include <HAPIHapticShape.h>
#include <HapticForceEffect.h>
#include <Threads.h>
#include <AutoRefVector.h>

namespace H3D {

  /// \class HAPIHapticsDevice
  /// Base class for all haptic devices. 
  class HAPI_API HAPIHapticsDevice {
  public:
    typedef AutoRefVector< HAPIHapticShape > HapticShapeVector;
    typedef AutoRefVector< HapticForceEffect > HapticEffectVector;

    /// Destructor. Stops haptics rendering and remove callback functions.
    virtual ~HAPIHapticsDevice() {}

    /// Does all the initialization needed for the device before starting to
    /// use it.
    virtual void initDevice() = 0;

    virtual void enableDevice() = 0;

    virtual void disableDevice() = 0;

    /// Perform cleanup and let go of all device resources that are allocated.
    /// After a call to this function no haptic rendering can be performed on
    /// the device until the initDevice() function has been called again.
    virtual void releaseDevice() = 0;

    /// Perform haptic rendering for the given HapticShape instances. 
    /// HapticShape objects that are to be be rendered haptically must be 
    /// rendered with this function each scenegraph loop. 
    /// \param objects The haptic shapes to render.
    ///
    inline void addShape( HAPIHapticShape *shape ) {
      current_shapes.push_back( shape );
    }

    inline void removeShape( HAPIHapticShape *shape ) {
      current_shapes.erase( shape );
    }

    template< class InputIterator > 
    inline void addShapes( InputIterator begin, InputIterator end ) {
      current_shapes.insert( current_shapes.end(), begin, end );
    }

    inline void swapShapes( HapticShapeVector &shapes ) {
      current_shapes.swap( shapes );
    }

    virtual void clearShapes() {
      current_shapes.clear();
    }

    /// Perform haptic rendering for the given HapticForceEffect instances. 
    /// HapticForceEffect objects that are to be be rendered haptically must
    /// be rendered with this function each scenegraph loop.
    /// \param objects The haptic objects to render.
    ///
    inline void addEffect( HapticForceEffect *effect ) {
      current_force_effects.push_back( effect );
    }

    inline void removeEffect( HapticForceEffect *effect ) {
      current_force_effects.erase( effect );
    }

    template< class InputIterator > 
    inline void addEffects( InputIterator begin, InputIterator end ) {
      current_force_effects.insert( current_force_effects.end(), begin, end );
    }

    inline void swapEffects( HapticEffectVector &effects ) {
      current_force_effects.swap( effects );
    }

    virtual void clearEffects() {
      current_force_effects.clear();
    }
    
    /// Get the position of the haptics device. Only to be called in the 
    /// haptics loop.
    virtual Vec3d getPosition() = 0;

    /// Get the velocity of the haptics device. Only to be called in the 
    /// haptics loop.
    virtual Vec3d getVelocity() = 0;

    /// Get the orientation of the haptics device. Only to be called in the 
    /// haptics loop.
    virtual Rotation getOrientation() = 0;

    /// Returns true if main button on haptics device pressed. Only to be called in the 
    /// haptics loop.
    inline bool getButtonStatus( unsigned int button_nr ) {
      return getButtonStatus() & (1 << button_nr);
    }

    virtual H3DInt32 getButtonStatus() = 0;

    /// Send the force to render on the haptics device. Only to be called in the 
    /// haptics loop.
    inline void sendForce( const Vec3d &f ) {
      sendForceToDevice( f ); 
      current_force = f;
    }

    /// Send the torque to render on the haptics device. Only to be called in the 
    /// haptics loop.
    inline void sendTorque( const Vec3d &t ) {
      sendTorqueToDevice( t );
      current_torque = t;
    }
    
    void renderHapticsOneStep();

  protected:

    virtual void sendForceToDevice( const Vec3d &f ) = 0;
    virtual void sendTorqueToDevice( const Vec3d &t ) = 0;

    /// The thread that this haptics device loop is run in.
    PeriodicThreadBase *thread;

    // the force effects that are currently rendered in the realtime loop.
    // Should not be changed directly from the scenegraph loop but instead
    // use the renderEffects function to set the effects.
    HapticEffectVector current_force_effects;

    // the force effects that was current in the last scene graph loop.
    HapticEffectVector last_force_effects;

    // the shapes that are currently being rendered in the realtime loop.
    HapticShapeVector current_shapes;

    unsigned int nr_haptics_loops;

    /// The time at the beginning of the last rendering loop
    TimeStamp last_loop_time;

    Vec3d proxy_position;
    Vec3d current_force;
    Vec3d current_torque;

  };
}

#endif
