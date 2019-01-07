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
/// \file HAPIHapticsDevice.h
/// \brief Header file for HAPIHapticsDevice.
///
//
//////////////////////////////////////////////////////////////////////////////

#ifndef __HAPIHAPTICSDEVICE_H__
#define __HAPIHAPTICSDEVICE_H__

#include <HAPI/HAPI.h>
#include <HAPI/HAPIHapticShape.h>
#include <HAPI/HAPIHapticsRenderer.h>
#include <HAPI/HAPIForceEffect.h>
#include <H3DUtil/Threads.h>
#include <H3DUtil/AutoRefVector.h>

#include <memory>
namespace HAPI {

  /// \ingroup AbstractClasses
  /// \class HAPIHapticsDevice
  /// \brief Base class for all haptics devices.
  /// 
  /// The functions that should be implemented by all subclasses are:
  /// - bool initHapticsDevice( int _thread_frequency = 1024 )
  /// - bool releaseHapticsDevice()
  /// - void updateDeviceValues( DeviceValues &dv, HAPITime dt )
  /// - void sendOutput( DeviceOutput &dv, HAPITime dt )
  ///
  /// In order to use a haptics device two things have to be done. \n
  /// 1. Initialize the device (initDevice()). \n
  /// 2. Enable the device (enableDevice()) \n
  ///
  /// When a device has been initialized it will be ready to start receiving
  /// forces and updating position values, but it will not start doing these 
  /// things until it is enabled.
  class HAPI_API HAPIHapticsDevice {
  public:
    /// Type for the possible states a haptics device can be in. In order for 
    /// a haptics device to be able to send forces it has to be ENABLED.
    typedef enum {
      UNINITIALIZED, /// the device has not been intialized 
      INITIALIZED,   /// the device has been initialized but not enabled
      ENABLED        /// the device has been initialized and enabled
    } DeviceState;
    
    /// Type for different kinds of errors.
    typedef enum {
      SUCCESS = 0,   
      NOT_INITIALIZED, /// the device has not been initialized
      NOT_ENABLED,     /// the device has not been enabled
      FAIL             /// the operation has failed for another reason
                       /// use getLastErrorMsg() to get info about the error.
    } ErrorCode;

    std::string profiled_result_haptic[2];

    /// \struct DeviceValues
    /// \brief Struct for holding a snapshot of current values of different
    /// properties of the haptics device.
    struct HAPI_API DeviceValues {
      DeviceValues():
        button_status( 0 ),
        user_data( NULL ),
        dof7_force( 0 ),
        dof7_angle( 0 ) {}

      /// The force currently being rendered
      Vec3 force;
      /// The torque currently being rendered
      Vec3 torque;
      /// The position of the haptics device
      Vec3 position;
      /// The velocity of the haptics device.
      Vec3 velocity;
      /// The orientation of the haptics device.
      Rotation orientation;
      /// The status of the buttons.
      HAPIInt32 button_status;
      /// Extra data that can be used be developers when
      /// supporting new device types.
      void *user_data;
      
      /// The angular velocity in radius around each axis.
      Vec3 angular_velocity;

      /// The gripper force for 7DoF devices, to for example a gripper. Defaults to 0.
      HAPIFloat dof7_force;
      /// The angle for 7Dof devices, for example a gripper. Defaults to 0.
      HAPIFloat dof7_angle;
    };

    typedef H3DUtil::AutoRefVector< HAPIHapticShape > HapticShapeVector;
    typedef H3DUtil::AutoRefVector< HAPIForceEffect > HapticEffectVector;

    /// Constructor.
    HAPIHapticsDevice() :
      thread( NULL ),
      tmp_switching_effects( false ),
      switching_effects( false ),
      tmp_switch_effects_duration( 0 ),
      switch_effects_duration( 0 ),
      time_in_last_loop( -1 ),
      device_state( UNINITIALIZED ),
      delete_thread( false ),
      setup_haptic_rendering_callback( true ),
      haptic_rendering_cb_handle( -1 ),
      haptics_rate( 0 ),
      nr_haptics_loops( 0 ),
      last_force_effect_change( 0 ),
      error_handler( new DefaultErrorHandler ),
      force_limit( -1 ),
      torque_limit ( -1 ),
      force_scale ( 1 ),
      transfer_objects_active ( false ),
      always_transfer_objects ( true ),
      ts_force_scale( 1.0 ) {
      setHapticsRenderer( NULL );
      haptic_rendering_callback_data = this;
      // This value is choosen ad hoc as fairly "standard".
      // All devices should have a correct value.
      max_stiffness = 700;
    }
    
    /// Destructor. Stops haptics rendering and remove callback functions.
    virtual ~HAPIHapticsDevice() {}

    ////////////////////////////////////////////////////////////////////
    // Device handling functions.
    //

    /// Does all the initialization needed for the device before starting to
    /// use it.
    /// \param _thread_frequency is the desired haptic frequency. Check
    /// comment for the function initHapticsDevice() of each haptics device
    /// class to know what the values might do for that class.
    /// \returns SUCCESS if device was successfully initialized.
    virtual ErrorCode initDevice( int _thread_frequency = 1024 );

    /// Enable the device. Positions can be read and force can be sent.
    inline virtual ErrorCode enableDevice() {
      if( device_state == UNINITIALIZED ) {
        return NOT_INITIALIZED;
      }

      device_state = ENABLED;
      return SUCCESS;
    }

    /// Temporarily disable the device. Forces sent will be ignored and
    /// positions and orientation will stay the same as previous values.
    /// \returns SUCCESS if device was enabled and function succeeded.
    /// NOT_INITALIZED if device was not initalized.
    inline virtual ErrorCode disableDevice() {
      if( device_state == UNINITIALIZED ) {
        return NOT_INITIALIZED;
      }

      sendForce( Vec3( 0, 0, 0 ) );
      sendTorque( Vec3( 0, 0, 0 ) );
                 
      device_state = INITIALIZED;
      return SUCCESS;
    }

    /// Perform cleanup and let go of all device resources that are allocated.
    /// After a call to this function no haptic rendering can be performed on
    /// the device until the initDevice( _thread_frequency = 1024 ) function
    /// has been called again.
    /// \returns SUCCESS if device was released. NOT_INITALIZED if device was
    /// not initalized and FAIL if releasing device failed.
    inline virtual ErrorCode releaseDevice() {
      if( device_state == UNINITIALIZED ) {
        return NOT_INITIALIZED;
      }
      
      for( unsigned int i = 0; i < haptics_renderers.size(); ++i ) {
        if( haptics_renderers[i] ) {
          haptics_renderers[i]->releaseRenderer( this );
          haptics_renderers[i]->cleanUpStuff( this );
        }
      }

      if( thread && haptic_rendering_cb_handle != -1 )
        thread->removeAsynchronousCallback( haptic_rendering_cb_handle );

      if( thread && delete_thread ) {
        delete thread;
        thread = NULL;
        delete_thread = false;
      }

      if( !releaseHapticsDevice() ) {
        return FAIL;
      }

      device_state = UNINITIALIZED;
      return SUCCESS;
    }      

    ////////////////////////////////////////////////////////////////////
    // Functions for adding and removing shapes to be rendered by the
    // haptics renderer
    //

    /// Add a HAPIHapticShape to be rendered haptically.
    ///
    /// Adding and removing shapes and force effects should all be
    /// done from the same ("main") thread. To move the objects to the
    /// haptics thread call transferObjects(), also from the main thread.
    ///
    /// \param shape The haptic shape to render.
    /// \param layer The haptic layer to add the shape to.
    inline void addShape( HAPIHapticShape *shape, 
                          unsigned int layer = 0 ) {

      // Wait for previous callback to complete
      transfer_objects_cond.lock ();
      while ( transfer_objects_active ) {
        transfer_objects_cond.wait();
      }
      transfer_objects_cond.unlock ();

      assureSize( layer );
      tmp_shapes[layer].push_back( shape );
    }
    
    
    /// Set the HapticShapes to be rendered.
    ///
    /// Adding and removing shapes and force effects should all be
    /// done from the same ("main") thread. To move the objects to the
    /// haptics thread call transferObjects(), also from the main thread.
	  ///
    /// \param shapes The haptic shapes to render.
    /// \param layer The haptic layer to add the shape to.
    inline void setShapes( const HapticShapeVector &shapes, 
                           unsigned int layer = 0 ) {

      // Wait for previous callback to complete
      transfer_objects_cond.lock ();
      while ( transfer_objects_active ) {
        transfer_objects_cond.wait();
      }
      transfer_objects_cond.unlock ();

      assureSize( layer );
      HapticShapeVector v = shapes;
      v.swap( tmp_shapes[layer] );
    }

    /// Get the shapes currently used
    ///
    /// Adding and removing shapes and force effects should all be
    /// done from the same ("main") thread. To move the objects to the
    /// haptics thread call transferObjects(), also from the main thread.
	  ///
    /// \param layer The haptic layer to get the shapes from.
    inline HapticShapeVector getShapes( unsigned int layer = 0 ) {
      HapticShapeVector return_shapes;
      
      // Wait for previous callback to complete
      transfer_objects_cond.lock ();
      while ( transfer_objects_active ) {
        transfer_objects_cond.wait();
      }
      transfer_objects_cond.unlock ();

      assureSize( layer );
      return_shapes= tmp_shapes[layer];

      return return_shapes;
    }

    /// Remove a HAPIHapticShape from the shapes being rendered.
    ///
    /// Adding and removing shapes and force effects should all be
    /// done from the same ("main") thread. To move the objects to the
    /// haptics thread call transferObjects(), also from the main thread.
    ///
    /// \param shape The haptic shape to remove.
    /// \param layer The haptic layer to remove the shape from.
    inline void removeShape( HAPIHapticShape *shape,
                             unsigned int layer = 0 ) {
      // Wait for previous callback to complete
      transfer_objects_cond.lock ();
      while ( transfer_objects_active ) {
        transfer_objects_cond.wait();
      }
      transfer_objects_cond.unlock ();

      assureSize( layer );
      tmp_shapes[layer].erase( shape );
    }
    
    /// Add all shapes between [begin, end)
    ///
    /// Adding and removing shapes and force effects should all be
    /// done from the same ("main") thread. To move the objects to the
    /// haptics thread call transferObjects(), also from the main thread.
	  ///
    /// \param begin Iterator of where to begin getting shapes.
    /// \param end Iterator of where to stop getting shapes.
    /// \param layer The haptic layer to add shapes to
    template< class InputIterator > 
    inline void addShapes( InputIterator begin, InputIterator end,
                           unsigned int layer = 0 ) {

      // Wait for previous callback to complete
      transfer_objects_cond.lock ();
      while ( transfer_objects_active ) {
        transfer_objects_cond.wait();
      }
      transfer_objects_cond.unlock ();

      assureSize( layer );
      tmp_shapes[layer].insert( tmp_shapes[0].end(), begin, end );
    }

    /// Swap the vector of shapes currently being rendered with the
    ///
    /// Adding and removing shapes and force effects should all be
    /// done from the same ("main") thread. To move the objects to the
    /// haptics thread call transferObjects(), also from the main thread.
	  ///
    /// given vector, replacing all shapes being rendered.
    /// \param shapes The haptic shapes to render.
    /// \param layer The haptic layer to add shapes to
    inline void swapShapes( HapticShapeVector &shapes, 
                            unsigned int layer = 0 ) {

      // Wait for previous callback to complete
      transfer_objects_cond.lock ();
      while ( transfer_objects_active ) {
        transfer_objects_cond.wait();
      }
      transfer_objects_cond.unlock ();
      
      assureSize( layer );
      tmp_shapes[layer].swap( shapes );
    }

    /// Remove all HAPIHapticShape objects that are currently being rendered.
    ///
    /// Adding and removing shapes and force effects should all be
    /// done from the same ("main") thread. To move the objects to the
    /// haptics thread call transferObjects(), also from the main thread.
	  ///
    /// \param layer The haptic layer to clear
    inline void clearShapes( unsigned int layer = 0 ) {
      // Wait for previous callback to complete
      transfer_objects_cond.lock ();
      while ( transfer_objects_active ) {
        transfer_objects_cond.wait();
      }
      transfer_objects_cond.unlock ();

      assureSize( layer );
      tmp_shapes[layer].clear();
    }

    /// Add a HapticForceEffect to be rendered.
    ///
    /// Adding and removing shapes and force effects should all be
    /// done from the same ("main") thread. To move the objects to the
    /// haptics thread call transferObjects(), also from the main thread.
	  ///
    /// \param effect The haptic shapes to render.
    /// \param fade_in_time The time until the effect is rendered at full
    /// strength.
    inline void addEffect( HAPIForceEffect *effect,
                           HAPITime fade_in_time = 0 ) {

      // Wait for previous callback to complete
      transfer_objects_cond.lock ();
      while ( transfer_objects_active ) {
        transfer_objects_cond.wait();
      }
      transfer_objects_cond.unlock ();

      tmp_current_force_effects.push_back( effect );
      added_effects_indices.push_back(
        std::make_pair( (unsigned int) tmp_current_force_effects.size() - 1, fade_in_time ) );
    }

    /// Set the HapticForceEffects to be rendered.
    ///
    /// Adding and removing shapes and force effects should all be
    /// done from the same ("main") thread. To move the objects to the
    /// haptics thread call transferObjects(), also from the main thread.
	  ///
    /// \param effects The haptic shapes to render.
    /// \param _switch_effects_duration The time until the new effects have
    /// completely replaced the old ones. During this time the old will be
    /// interpolated with a fraction and the new will be interpolated with
    /// 1 - fraction.
    inline void setEffects( const HapticEffectVector &effects,
                            HAPITime _switch_effects_duration = 0 ) {
      
      // Wait for previous callback to complete
      transfer_objects_cond.lock ();
      while ( transfer_objects_active ) {
        transfer_objects_cond.wait();
      }
      transfer_objects_cond.unlock ();

      tmp_current_force_effects = effects;
      tmp_switch_effects_duration = _switch_effects_duration;
      tmp_switching_effects = true;
      force_effects_to_remove.clear();
      added_effects_indices.clear();
    }

    /// Get the shapes currently used
    ///
    /// Adding and removing shapes and force effects should all be
    /// done from the same ("main") thread. To move the objects to the
    /// haptics thread call transferObjects(), also from the main thread.
	  ///
    inline HapticEffectVector getEffects() {
      HapticEffectVector return_force_effects;
      
      // Wait for previous callback to complete
      transfer_objects_cond.lock ();
      while ( transfer_objects_active ) {
        transfer_objects_cond.wait();
      }
      transfer_objects_cond.unlock();
      
      return_force_effects = tmp_current_force_effects;

      return return_force_effects;
    }

    /// Remove a force effect so that it is not rendered any longer.
    ///
    /// Adding and removing shapes and force effects should all be
    /// done from the same ("main") thread. To move the objects to the
    /// haptics thread call transferObjects(), also from the main thread.
	  ///
    inline void removeEffect( HAPIForceEffect *effect,
                              HAPITime fade_out_time = 0 ) {
      
      // Wait for previous callback to complete
      transfer_objects_cond.lock ();
      while ( transfer_objects_active ) {
        transfer_objects_cond.wait();
      }
      transfer_objects_cond.unlock();

      unsigned int i = 0;
      for( HapticEffectVector::const_iterator j =
             tmp_current_force_effects.begin();
           j != tmp_current_force_effects.end(); ++j, ++i )
        if( effect == (*j) )
          break;

      if( i < tmp_current_force_effects.size() ) {
        force_effects_to_remove.push_back( std::make_pair( effect, fade_out_time ));
        for( std::vector< std::pair< unsigned int, HAPITime > >::iterator j =
               added_effects_indices.begin();
             j != added_effects_indices.end(); ++j ) {
          if( i == (*j).first ) {
            added_effects_indices.erase( j );
            for( std::vector< std::pair< unsigned int, HAPITime > >::iterator k =
                   added_effects_indices.begin();
                 k != added_effects_indices.end(); ++k ) {
              --((*k).first);
            }
            break;
          }
        }
      }
      tmp_current_force_effects.erase( effect );
    }



    /// Add all effects between [begin, end)
    ///
    /// Adding and removing shapes and force effects should all be
    /// done from the same ("main") thread. To move the objects to the
    /// haptics thread call transferObjects(), also from the main thread.
	  ///
    template< class InputIterator > 
    inline void addEffects( InputIterator begin, InputIterator end ) {

      // Wait for previous callback to complete
      transfer_objects_cond.lock ();
      while ( transfer_objects_active ) {
        transfer_objects_cond.wait();
      }
      transfer_objects_cond.unlock();
      
      tmp_current_force_effects.insert( tmp_current_force_effects.end(), begin, end );
    }

    /// Swap the vector of effects currently being rendered with the
    /// given vector, replacing all effects being rendered.
    ///
    /// Adding and removing shapes and force effects should all be
    /// done from the same ("main") thread. To move the objects to the
    /// haptics thread call transferObjects(), also from the main thread.
	  ///
    /// \param effects The new effects which should be rendered on the
    /// haptics device.
    /// \param _switch_effects_duration Duration in seconds before the old
    /// effects are fully replaced by the new. Linear interpolation between
    /// old and new effects. The start time of the interpolation is the call
    /// to transferObjects().
    inline void swapEffects( HapticEffectVector &effects,
                             HAPITime _switch_effects_duration = 0 ) {

      // Wait for previous callback to complete
      transfer_objects_cond.lock ();
      while ( transfer_objects_active ) {
        transfer_objects_cond.wait();
      }      
      transfer_objects_cond.unlock();

      tmp_current_force_effects.swap( effects );
      tmp_switch_effects_duration = _switch_effects_duration;
      tmp_switching_effects = true;
      force_effects_to_remove.clear();
      added_effects_indices.clear();
    }

    /// Remove all HAPIForceEffect objects that are currently being rendered.
    ///
    /// Adding and removing shapes and force effects should all be
    /// done from the same ("main") thread. To move the objects to the
    /// haptics thread call transferObjects(), also from the main thread.
	  ///
    inline void clearEffects() {
      
      // Wait for previous callback to complete
      transfer_objects_cond.lock ();
      while ( transfer_objects_active ) {
        transfer_objects_cond.wait();
      }
      transfer_objects_cond.unlock();

      tmp_current_force_effects.clear();
      tmp_switching_effects = true;
      tmp_switch_effects_duration = 0;
      force_effects_to_remove.clear();
      added_effects_indices.clear();
    }
    
    /// Transfer all current haptic objects to be rendered by the haptics
    /// renderer.
    ///
    /// Adding and removing shapes and force effects should all be
    /// done from the same ("main") thread. To move the objects to the
    /// haptics thread call transferObjects(), also from the main thread.
	  ///
    virtual void transferObjects();

    //////////////////////////////////////////////////////////////
    // Functions for getting/settings device values
    //
    
    /// \brief Set the position calibration matrix, i.e. the transform matrix
    /// from the local device coordinate space to HAPI coordinate space. Also
    /// sets the inverse matrix.
    inline void setPositionCalibration( const Matrix4 &m,
                                        const Matrix4 &m_inv ) {
      device_values_lock.lock();
      position_calibration = m;
      position_calibration_inverse = m_inv;
      device_values_lock.unlock();
    }

    /// \brief Set the position calibration matrix, i.e. the transform matrix
    /// from the local device coordinate space to HAPI coordinate space.
    inline void setPositionCalibration( const Matrix4 &m ) {
      device_values_lock.lock();
      position_calibration = m;
      position_calibration_inverse = m.inverse();
      device_values_lock.unlock();
    }

    /// Get the current position calibration matrix    
    inline const Matrix4 &getPositionCalibration() { 
      return position_calibration;
    }

    /// Get the inverse of the current position calibration matrix    
    inline const Matrix4 &getPositionCalibrationInverse() { 
      return position_calibration_inverse;
    }

    /// Set the orientation calibration.
    inline void setOrientationCalibration( const Rotation &r ) {
      device_values_lock.lock();
      orientation_calibration = r;
      device_values_lock.unlock();
    }

    /// Get the current orientation calibration.
    inline const Rotation &getOrientationCalibration() {
      return orientation_calibration;
    }

    /// Get the current device state.
    inline DeviceState getDeviceState() {
      return device_state;
    }
#ifdef HAVE_PROFILER
    /// Get the current profiled result
    inline std::string getProfiledResult() {
      std::string temp1 = "";
      std::string temp2 = "";
      profiled_result_lock.lock();
      temp1 = profiled_result_haptic[0];
      temp2 = profiled_result_haptic[1];
      profiled_result_lock.unlock();
      if(!temp1.empty()&&!temp2.empty())
      {
        return "haptic rendering profiling result: \n" + temp1+temp2;
      }
      return "haptic rendering profiling result: \n";
    }
#endif
    /// Get the current device values in device coordinates.
    inline DeviceValues getRawDeviceValues() {
      device_values_lock.lock();
      DeviceValues dv = current_raw_device_values;
      device_values_lock.unlock();
      return dv;
    }

    /// Get the device values from the last loop in device coordinates.
    inline DeviceValues getLastRawDeviceValues() {
      device_values_lock.lock();
      DeviceValues dv = last_raw_device_values;
      device_values_lock.unlock();
      return dv;
    }

    /// Get the current device values in world coordinates.
    inline DeviceValues getDeviceValues() {
      device_values_lock.lock();
      DeviceValues dv = current_device_values;
      device_values_lock.unlock();
      return dv;
    }
    
    /// Get the device values from the last loop in world coordinates.
    inline DeviceValues getLastDeviceValues() {
      device_values_lock.lock();
      DeviceValues dv = last_device_values;
      device_values_lock.unlock();
      return dv;
    }

    /// \brief Get the position of the haptics device without the calibration
    /// matrix applied, i.e. coordinates in metres.
    inline Vec3 getRawPosition() {
      return getRawDeviceValues().position;
    }

    /// Get the velocity of the haptics device without the calibration
    /// matrix applied. 
    inline Vec3 getRawVelocity() {
      return getRawDeviceValues().velocity;
    }

    /// Get the orientation of the haptics device without the calibration
    /// matrix applied. 
    inline Rotation getRawOrientation() {
      return getRawDeviceValues().orientation;
    }

    /// Get the position of the haptics device with the calibration matrix 
    /// applied. 
    inline Vec3 getPosition() {
      return getDeviceValues().position;
    }

    /// Get the velocity of the haptics device with the calibration matrix 
    /// applied. 
    inline Vec3 getVelocity() {
      return getDeviceValues().velocity;
    }

    /// Get the orientation of the haptics device with the calibration matrix 
    /// applied. 
    inline Rotation getOrientation() {
      return getDeviceValues().orientation;
    }

    /// Get the button status. Bit 0 is button 0, bit 1 is button 1,..
    /// A 1 in the bit position indicates that the button is pressed.
    inline HAPIInt32 getButtonStatus() {
      return getDeviceValues().button_status;
    }

    /// Get the button status for a specified button. true means
    /// that is is pressed.
    inline bool getButtonStatus( unsigned int button_nr ) {
      return (getButtonStatus() & (1 << button_nr)) != 0;
    }

    /// Get the force currently rendered by the haptics device in device
    /// coordinates.
    inline Vec3 getRawForce() {
      return getRawDeviceValues().force;
    }

    /// Get the torque currently rendered by the haptics device in device 
    /// coordinates.
    inline Vec3 getRawTorque() {
      return getRawDeviceValues().torque;
    }

    /// Get the force currently rendered by the haptics device in world
    /// coordinates.
    inline Vec3 getForce() {
      return getDeviceValues().force;
    }

    /// Get the torque currently rendered by the haptics device in world 
    /// coordinates.
    inline Vec3 getTorque() {
      return getDeviceValues().torque;
    }

    /// Get the position of the haptics device from the last loop in device
    /// coordinates. 
    inline Vec3 getLastRawPosition() {
      return getLastRawDeviceValues().position;
    }

    /// Get the velocity of the haptics device from the last loop in device
    /// coordinates. 
    inline Vec3 getLastRawVelocity() {
      return getLastRawDeviceValues().velocity;
    }

    /// Get the orientation of the haptics device from the last loop in device
    /// coordinates. 
    inline Rotation getLastRawOrientation() {
      return getLastRawDeviceValues().orientation;
    }

    /// Get the force rendered by the haptics device in the last loop in device
    /// coordinates.
    inline Vec3 getLastRawForce() {
      return getLastRawDeviceValues().force;
    }

    /// Get the torque rendered by the haptics device in the last loop in 
    /// device coordinates.
    inline Vec3 getLastRawTorque() {
      return getLastRawDeviceValues().torque;
    }

    /// Get the position of the haptics device from the last loop in world
    /// coordinates. 
    inline Vec3 getLastPosition() {
      return getLastDeviceValues().position;
    }
    
    /// Get the velocity of the haptics device from the last loop in world
    /// coordinates. 
    inline Vec3 getLastVelocity() {
      return getLastDeviceValues().velocity;
    }

    /// Get the orientation of the haptics device from the last loop in world
    /// coordinates. 
    inline Rotation getLastOrientation() {
      return getLastDeviceValues().orientation;
    }

    /// Get the force rendered by the haptics device in the last loop in world
    /// coordinates.
    inline Vec3 getLastForce() {
      return getLastDeviceValues().force;
    }

    /// Get the torque rendered by the haptics device in the last loop in world
    /// coordinates.
    inline Vec3 getLastTorque() {
      return getLastDeviceValues().torque;
    }

    /// Get the button status from the last loop.
    inline HAPIInt32 getLastButtonStatus() {
      return getLastDeviceValues().button_status;
    }
    
    /// Get the button status from the last loop.
    inline bool getLastButtonStatus( unsigned int button_nr ) {
      return (getLastButtonStatus() & (1 << button_nr)) != 0;
    }

    /// Set the HAPIHapticsRenderer to use to render the HAPIHapticShapes
    /// specified for a specific layer.
    inline void setHapticsRenderer( HAPIHapticsRenderer *r, 
                                    unsigned int layer = 0 ) {


      
      if( haptics_renderers.size() < layer + 1 ) {
        renderer_change_lock.lock();
        haptics_renderers.resize( layer + 1, NULL );
        renderer_change_lock.unlock();
      }

      if( device_state != UNINITIALIZED ) {
        if( haptics_renderers[layer] ) {
          haptics_renderers[layer]->releaseRenderer( this );
        }
        if( r )
          r->initRenderer( this );
      }

      renderer_change_lock.lock();
      // Give ownership to temporary auto_ptr to get correct deletion of 
      // the renderer that is replaced since the auto_ptr_vector does not
      // take care of this.
      std::auto_ptr< HAPIHapticsRenderer > temp_rendr( haptics_renderers[ layer ] );
      haptics_renderers[ layer ] = r;
      renderer_change_lock.unlock();
    }

    /// Return the number of layers that have had shapes or has renderers
    /// added to it.
    inline unsigned int nrLayers() {
      return (unsigned int) haptics_renderers.size();
    }

    /// Get the currently used HAPIHapticsRenderer for a layer.
    inline HAPIHapticsRenderer *getHapticsRenderer( unsigned int layer = 0 ) {
      if( haptics_renderers.size() < layer + 1 ) return NULL;
      else return haptics_renderers[ layer ];
    }

    /// Get the error message from the latest error.
    inline const std::string &getLastErrorMsg() {
      return last_error_message;
    }

    /// This is the base class for error handlers for haptics devices. The ErrorHandler
    /// class handles all haptics device related errors from the haptics loop. Examples
    /// being errors/warnings such as maximum force exceeded, temperature errors etc.
    /// The errors codes depends on which device is used and are from the underlying
    /// driver. In order to create your own error handler, subclass from this class 
    /// and specialize the handleError function and call setErrorHandler on a 
    /// HAPIHapticsDevice with an instance of your error handler. Be aware that
    /// the function will be called in the haptics communication thread so be sure that
    /// your error handling functions are thread safe.
    class HAPI_API ErrorHandler: public H3DUtil::RefCountedClass {
    public:
      /// Error handling function. Called upon haptics device error during haptics
      /// rendering.
      /// \param hd The haptics device the error occured on.
      /// \param internal_error_code The internal error code reported from the haptics
      /// device driver.
      /// \param error_string Human readable text describing the error.
      virtual void handleError( HAPIHapticsDevice *hd, 
                                long internal_error_code,
                                std::string error_string ) = 0;
    };
    
    /// The default error handler that is used by haptics devices. It outputs
    /// an error message to the Console.
    class HAPI_API DefaultErrorHandler: public ErrorHandler {
    public:
      /// Outputs a Console error message.
      virtual void handleError( HAPIHapticsDevice *hd, 
                                long internal_error_code,
                                std::string error_string ) {
        H3DUtil::Console(H3DUtil::LogLevel::Error) << "Haptics device error: " << error_string
                              << " (error code " << internal_error_code 
                              << ")" << std::endl;
      }
    };
    
    /// Get the currently installed error handler.
    ErrorHandler *getErrorHandler() {
      return error_handler.get();
    }
    
    /// Set the error handler.
    virtual void setErrorHandler( ErrorHandler *_error_handler ) {
      error_handler.reset( _error_handler );
    }
    
    /// Get the current actual update rate in loops per second
    inline unsigned int getHapticsRate() { return haptics_rate; }

    /// Get the time spent in the last haptics loop(in seconds)
    /// If the time returned is -1 then no haptics loop has yet
    /// been completed.
    inline HAPITime getTimeSpentInLastLoop() {
      return time_in_last_loop;
    }

    /// Get the thread that is used to run this haptics device.
    inline H3DUtil::PeriodicThreadBase *getThread() {
      return thread;
    }

    // Should return the maximum stiffness (N/m) the haptics device can
    // handle. returns -1 if for some reason there is no such value.
    HAPIFloat getMaxStiffness() {
      return max_stiffness;
    }

    /// Get the current maximum force limit.
    inline HAPIFloat getForceLimit() { return force_limit; }

    /// Get the current maximum torque limit.
    inline HAPIFloat getTorqueLimit() { return torque_limit; }

    /// Get the current force scale
    inline HAPIFloat getForceScale() {
      HAPIFloat scale;
      device_values_lock.lock();
      scale = force_scale;
      device_values_lock.unlock();
      return scale;
    }

    /// Set the maximum force limit. The maximum force limit(in Newton) for the 
    /// device. Any forces generated that are higher than the limit will be 
    /// clamped to the limit before sent to the device. A negative value means
    /// no limit.
    inline void setForceLimit( HAPIFloat limit ) { force_limit = limit; }

    /// Set the maximum torque limit. The maximum torque limit(in Nm) for the 
    /// device. Any torques generated that are higher than the limit will be 
    /// clamped to the limit before sent to the device. A negative value means
    /// no limit
    inline void setTorqueLimit( HAPIFloat limit ) { torque_limit = limit; }

    /// Set the scaling of the output force. The forces generated will
    /// be multiplied by the value.
    inline void setForceScale( HAPIFloat scale ) {
      device_values_lock.lock();
      force_scale = scale;
      device_values_lock.unlock();
    }

    /// If true then shapes and force effects are always transfered to the haptic thread
    /// when transferObjects() is called. If false then they are only transfered if they have
    /// been changed.
    ///
    /// Examining the objects to detect changes will consume more time, but transfering the objects
    /// is also time consuming, especially if the haptics thread rate fluctuates. Therefor the optimal
    /// setting is application dependant.
	  ///
	  /// The default is to always transfer objects.
	  ///
    void setAlwaysTransferObjects ( bool _alwaysTransferObjects ) { always_transfer_objects= _alwaysTransferObjects; }

    // The following is part of the database of available haptics devices.
    typedef HAPIHapticsDevice*( *CreateInstanceFunc)(); 

    template< class N >
    static HAPIHapticsDevice *newInstance() { return new N; }

    /// Struct used to register a class to the registered haptics devices.
    /// For example on how to use this class see source code of
    /// FalconHapticsDevice.
    struct HAPI_API HapticsDeviceRegistration{
    public:
      /// Constructor.
      /// \param _name The name of the device.
      /// \param _create A function for creating an instance of the device
      /// class.
      /// \param _libs_to_support A list of names of dlls that must exist on
      /// the system for this device to be supported. Only needed for devices
      /// that should be supported in Microsoft Windows.
      HapticsDeviceRegistration( const std::string &_name,
                                 CreateInstanceFunc _create,
                                 std::list< std::string > _libs_to_support = std::list<std::string >() ):
      name( _name ),
      create_func( _create ),
      libs_to_support( _libs_to_support )
      {
        
        if( !initialized ) {
          HAPIHapticsDevice::registered_devices.reset( 
            new std::list< HapticsDeviceRegistration > );
          initialized = true;
        }
        HAPIHapticsDevice::registerDevice( *this );
      }

      std::string name;
      CreateInstanceFunc create_func;
      std::list< std::string > libs_to_support;
    };
#ifdef __BORLANDC__
    friend struct HapticsDeviceRegistration;
#endif

    /// Register a haptics device to the database.
    /// \param name The name of the device
    /// \param create A function for creating an instance of that class.
    /// \param libs_to_support A list of strings with libraries that is needed
    /// to be supported by this device (dlls, so).
    static void registerDevice( const std::string &name,
                                  CreateInstanceFunc create,
                                  std::list< std::string > libs_to_support ) {
      registerDevice( HapticsDeviceRegistration( name, create, 
                                                 libs_to_support ) );
    }

    /// Register a haptics device that can then be returned by 
    /// getSupportedFileReader().
    static void registerDevice( const HapticsDeviceRegistration &fr ) {
      registered_devices->push_back( fr );
    }

    // Creating a new auto_ptr local for this node, because 
    // registered_devices caused a memory leak and because
    // of the order of setting the static variables the auto_ptr's
    // constructor resets the auto_ptr to 0 eventhough the 
    // registered_devices has been initialized, and therefore
    // cause an error making it impossible to use the standard auto_ptr.
    template<class T>
    class local_auto_ptr{
    private:
      T* ap;    // refers to the actual owned object (if any)
    public:
      typedef T element_type;

      // constructor
      explicit local_auto_ptr (T* ptr = 0) {
        if(!initialized){
          ap=ptr;
        }
      }
      
      // copy constructors (with implicit conversion)
      // - note: nonconstant parameter
      local_auto_ptr (local_auto_ptr& rhs) throw() : ap(rhs.release()) { }

      template<class Y>
      local_auto_ptr (local_auto_ptr<Y>& rhs) throw() : ap(rhs.release()) { }
      
      // assignments (with implicit conversion)
      // - note: nonconstant parameter
      local_auto_ptr& operator= (local_auto_ptr& rhs) throw(){
        if(!initialized){  
          reset(rhs.release());
          return *this;
        }
      }
      template<class Y>
      local_auto_ptr& operator= (local_auto_ptr<Y>& rhs) throw(){
        if(!initialized){
          reset(rhs.release());
          return *this;
        }
      }

      // destructor
      ~local_auto_ptr() throw(){
        delete ap;
      }

      // value access
      T* get() const throw(){
        return ap;
      }
      T& operator*() const throw(){
        return *ap;
      }
      T* operator->() const throw(){
        return ap;
      }

      // release ownership
      T* release() throw(){
        if(!initialized){
          T* tmp(ap);
          ap = 0;
          return tmp;
        }
      }

      // reset value
      void reset (T* ptr=0) throw(){
        if(!initialized){
          if (ap != ptr){
            delete ap;
            ap = ptr;
          }
        }
      }
    };

    static local_auto_ptr< std::list< HapticsDeviceRegistration > >
    registered_devices;
    static bool initialized;

  protected:

    inline void assureSize( unsigned int i ) {
      if( tmp_shapes.size() < i + 1 ) tmp_shapes.resize( i + 1 );
      if( haptics_renderers.size() < i + 1 ) 
        haptics_renderers.resize( i + 1, NULL );
    }

    /// Send the force to render on the haptics device in device coordinates. 
    inline void sendRawForce( const Vec3 &f ) {
      if( device_state == ENABLED ) {
        device_values_lock.lock();
        output.force = f;
        device_values_lock.unlock();
      }
    }

    /// Send the torque to render on the haptics device in device coordinates. 
    inline void sendRawTorque( const Vec3 &t ) {
      if( device_state == ENABLED ) {
        device_values_lock.lock();
        output.torque = t;
        device_values_lock.unlock();
      }
    }

    /// Send the force to render on the haptics device  in world coordinates. 
    inline void sendForce( const Vec3 &f ) {
      if( device_state == ENABLED ) {
        device_values_lock.lock();
        output.force = position_calibration_inverse.getRotationPart() * f;
        device_values_lock.unlock();
      }
    }

    /// Send the torque to render on the haptics device in world coordinates. 
    inline void sendTorque( const Vec3 &t ) {
      if( device_state == ENABLED ) {
        device_values_lock.lock();
        output.torque = -orientation_calibration * t;
        device_values_lock.unlock();
      }
    }

    /// Output that is sent to the haptics device to render.
    struct HAPI_API DeviceOutput {
      Vec3 force;  /// The force in Newtons
      Vec3 torque; /// The torque in Newtons/mm
      HAPIFloat dof7_force; /// 7dof force in Newtons. For example for a gripper.
    };

    /// Updates the current_device_values member to contain
    /// current values.
    /// \param dt Time since last haptic loop.
    void updateDeviceValues( HAPITime dt );
    
    /// Calculates instant velocity using current_raw_device_values
    /// and the device values sent as input.
    /// \param dv DeviceValues struct containing the current position
    /// and the velocity that should be set.
    /// \param dt Time since last call to this function (usually the time
    /// between two haptic loops).
    inline void calculateVelocity( DeviceValues &dv,
                                   HAPITime dt ) {
      dv.velocity = ( dv.position - current_raw_device_values.position ) / dt;
    }

    /// Sends the data in the output member to be rendered at
    /// the haptics device.
    /// \param dt Time since last call to this function.
    inline void sendOutput( HAPITime dt = 0 ) {
      if( device_state == ENABLED ) {
        device_values_lock.lock();
        sendOutput( output, dt );
        device_values_lock.unlock();
      }
    }

    /// Sets an error message that can be accessed later through
    /// getLastErrorMsg()
    inline void setErrorMsg( const std::string &err ) {
      last_error_message = err;
    }

    /// This function should be overriden by all subclasses of 
    /// HAPIHapticsDevice in order to fill in the given DeviceValues
    /// structure with current values of the devie.
    /// \param dv Contains values that should be updated.
    /// \param dt Time since last call to this function.
    virtual void updateDeviceValues( DeviceValues &dv,
                                     HAPITime dt ) {
      dv.force = output.force;
      dv.torque = output.torque;
      dv.dof7_force = output.dof7_force;
    }

    /// This function should be overridden by all subclasses of 
    /// HAPIHapticsDevice in order to send the values in the
    /// given DeviceOutput structure to the haptics device.
    /// \param dv Struct which contains force values to send to the haptics
    /// device.
    /// \param dt Time since last call to this function.
    virtual void sendOutput( DeviceOutput &dv,
                             HAPITime dt ) = 0;

    /// Initialize the haptics device.
    /// \param _thread_frequency is the desired haptic frequency. Check
    /// comment for the function initHapticsDevice() for each haptics device
    /// class to know what values can be set. By default
    /// 1024 is the maximum allowed frequency that can be specified. Setting
    /// this parameter to -1 means run as fast as possible. It is recommended
    /// to use the default value for most users.
    /// The system will try to match the frequency as close as possible but
    /// the actual frequency is dependent on the frequency of the timer on the 
    /// system. E.g on a Windows system the multimedia timers are used for
    /// synchronization. When run at its highest frequence this will have a clock
    /// cycle of 0.976 ms. This means that the highest frequency we can get is
    /// 1024. Since we only can get an event from the timer once for each ms, the
    /// possible frequences are 1024/x, where x is the number of milliseconds to run
    /// each loop in the thread, i.e. 1024, 512, 342, 256, 205 and so on.
    /// Some haptics devices uses other synchronization means than the RTC timer
    /// though and in those cases they might have different possible frequencies.
    /// \returns true if initialization succeeded.
    virtual bool initHapticsDevice( int _thread_frequency = 1024 ) = 0;

    /// Release all resources allocated to the haptics device.
    virtual bool releaseHapticsDevice() = 0;

    /// Returns true if and only if a shapes or force effects have been changed
    /// requiring a synchronisation with the haptics thread
    bool haveObjectsChanged ();

    /// The thread that this haptics device loop is run in.
    H3DUtil::PeriodicThreadBase *thread;

    // The force effects that are currently rendered in the haptics loop.
    // Only used in the haptics loop.
    HapticEffectVector current_force_effects;

    // A copy of the current_force_effects vector which is used to add and
    // remove force effect from the haptics loop. The force effects are copied
    // over to the haptics loop when transferObjects() is called.
    HapticEffectVector tmp_current_force_effects;

    // The forces to interpolate with if interpolation should be done.
    HapticEffectVector last_force_effects;

    // Flag used to know if interpolation should be done between
    // current_force_effects and last_force_effects. Set by setEffects
    // and swapEffects.
    bool switching_effects;

    // Copy of switching_effects not used in haptics loop.
    bool tmp_switching_effects;

    // The time it takes to fully switch to the new force effects when
    // using setForceEffects or swapForceEffects.
    HAPITime switch_effects_duration;

    // Copy of switch_effects_duration not used in haptics loop.
    HAPITime tmp_switch_effects_duration;

    // Used to know which effects was added through the addEffect function.
    std::vector< std::pair< unsigned int, HAPITime > > added_effects_indices;

    // Used to know which effects was removed through the removeEffect
    // function.
    std::vector< std::pair< HAPIForceEffect *, HAPITime > > force_effects_to_remove;

    // The time of the last call to setForceEffects or swapForceEffects.
    // Only set if switching_effects is true.
    HAPITime last_force_effect_change;

    // Struct used to calculate fractions when adding or removing effects
    // with the addEffect and removeEffect functions.
    struct HAPI_API PhaseInOut {
    public:
      // Default Constructor
      PhaseInOut(){};

      // Constructor.
      PhaseInOut( HAPITime _duration,
                  HAPITime _start_time,
                  bool _phase_in = true,
                  bool _set_max_fraction = false,
                  HAPIFloat _max_fraction = 0 ) :
      duration( _duration ),
      start_time( _start_time ),
      phase_in( _phase_in ),
      max_fraction( _max_fraction ),
      set_max_fraction( _set_max_fraction ) {}

      /// returns a fraction to use for interpolation. The return value
      /// is not normalized and could be bigger than 1 and smaller than 0.
      inline HAPIFloat getFraction( HAPITime now_time ) {
        HAPITime fraction = ( now_time - start_time ) / duration;
        if( phase_in ) {
          if( !set_max_fraction )
            set_max_fraction = true;
          max_fraction = fraction;
          return fraction;
        }
        else {
          fraction = 1 - fraction;
          if( set_max_fraction && fraction > max_fraction )
            return max_fraction;
          else return fraction;
        }
      }

      // Get max_fraction.
      inline HAPIFloat getMaxFraction() { return max_fraction; }

      // Set max_fraction;
      inline void setMaxFraction( HAPIFloat _max_fraction ) {
        max_fraction = _max_fraction;
        set_max_fraction = true;
      }

    protected:
      // The time it should take to reach fraction 1.
      HAPITime duration;

      // The time of creation.
      HAPITime start_time;

      // Determines whether to phase in or out. This means that getFraction
      // will return "fraction" or "1 - fraction" depending on this flag.
      bool phase_in;

      // Maximum fraction number that have been set when phasing in.
      // Used to determine the maximum value value when phasing out.
      HAPIFloat max_fraction;
      // true if a max_fraction is set.
      bool set_max_fraction;
    };

    typedef std::map< int, PhaseInOut > IndexTimeMap;
    // map to keep track of which force effects in current_force_effects that
    // are phased in.
    IndexTimeMap current_add_rem_effect_map;
    // map to keep track of which force effects in last_force_effects that
    // are phased in our out.
    IndexTimeMap last_add_rem_effect_map;

    // the shapes that are currently being rendered in the realtime loop.
    // One HapticEffectVector for each layer.
    std::vector< HapticShapeVector > current_shapes;

    // the values to send to the haptics device.
    DeviceOutput output;

    // lock for when updating device values/sending output
    H3DUtil::MutexLock device_values_lock;
#ifdef HAVE_PROFILER
    // lock for when updating device values/sending output
    H3DUtil::MutexLock profiled_result_lock;
#endif

    // lock for when changing haptics renderer
    H3DUtil::MutexLock renderer_change_lock;

    // container for shapes that will be transferred to the haptic
    // rendering loop through transferObjects.
    std::vector< HapticShapeVector > tmp_shapes ;

    /// The time at the beginning of the last rendering loop
    TimeStamp last_loop_time;

    /// The current error handler.
    H3DUtil::AutoRef< ErrorHandler > error_handler; 

    // Time in last loop, if -1 then no loop has been completed.
    HAPITime time_in_last_loop;

    DeviceState device_state;
    DeviceValues current_device_values;
    DeviceValues last_device_values;
    DeviceValues current_raw_device_values;
    DeviceValues last_raw_device_values;

    bool delete_thread;

    Matrix4 position_calibration;
    Matrix4 position_calibration_inverse;
    Rotation orientation_calibration;
    H3DUtil::AutoPtrVector< HAPIHapticsRenderer > haptics_renderers;
    std::string last_error_message;
    std::string device_name;

    // Thread safe variable containing copy of force_scale value
    HAPIFloat ts_force_scale;

    /// Callback function to render forces.
    static H3DUtil::PeriodicThread::CallbackCode
      hapticRenderingCallback( void *data );
#ifdef HAVE_PROFILER
    static H3DUtil::PeriodicThread::CallbackCode
      enableTimerCallback(void *data);
#endif
    /// If true then set up hapticRenderingCallback as its own callback.
    /// If false the hapticRenderingCallback functions has to be called
    /// explicity from somewhere.
    bool setup_haptic_rendering_callback;

    /// Callback handle to hapticRenderingCallback function
    /// -1 if not set.
    int haptic_rendering_cb_handle;

    /// Use this to send to hapticRenderingCallback when
    /// setup_haptic_rendering_callback is false. Needed to have
    /// anydevice call the callback correctly depending on
    /// setup_haptic_rendering_callback.
    void * haptic_rendering_callback_data;

    /// Callback function to transfer haptic objects to render to the 
    /// haptics loop. Used by transferObjects() function.
    static H3DUtil::PeriodicThread::CallbackCode
      transferObjectsCallback( void *data );

    friend class AnyHapticsDevice;
    friend class ClutchedHapticsDevice;

    unsigned int nr_haptics_loops;
    unsigned int haptics_rate;
    TimeStamp last_hr_update;

    /// The maximum force limit(in Newton) for the device. Any forces 
    /// generated that are higher than the limit will be clamped to the
    /// limit.
    HAPIFloat force_limit;

    /// The maximum torque limit(in Nm) for the device. Any torques 
    /// generated that are higher than the limit will be clamped to the
    /// limit.
    HAPIFloat torque_limit;

    /// The scale value that is multiplied with the forces applied
    /// to the haptics device
    HAPIFloat force_scale;

    /// The maximum stiffness the device can handle in N/mm.
    /// Should be set by each subclass to HAPIHapticsDevice.
    /// Default value is 700 N/m.
    HAPIFloat max_stiffness;
    
    /// A condition lock to signal that the transfer of objects is complete
    H3DUtil::ConditionLock transfer_objects_cond;
    bool transfer_objects_active;

    /// If true objects are always transfered to the haptics thread. Otherwise
    /// they are checked and only transfered if they have changed.
    bool always_transfer_objects;
  public:
    /// Get the angular velocity of the haptics device without the calibration
    /// matrix applied. 
    inline Vec3 getRawAngularVelocity() {
      return getRawDeviceValues().angular_velocity;
    }

    /// Get the angular velocity of the haptics device with the calibration matrix 
    /// applied. 
    inline Vec3 getAngularVelocity() {
      return getDeviceValues().angular_velocity;
    }

    /// Send the dof7 force to render on the haptics device. Since it does not have a direction
    /// only the raw function exists for now.
    inline void sendRawDof7Force( const HAPIFloat &f ) {
      if( device_state == ENABLED ) {
        device_values_lock.lock();
        output.dof7_force = f;
        device_values_lock.unlock();
      }
    }
  };
}

#endif
