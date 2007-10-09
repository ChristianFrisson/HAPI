//////////////////////////////////////////////////////////////////////////////
//    Copyright 2004-2007, SenseGraphics AB
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

#include <HAPI/HAPI.h>
#include <HAPI/HAPIHapticShape.h>
#include <HAPI/HAPIHapticsRenderer.h>
#include <HAPI/HAPIForceEffect.h>
#include <Threads.h>
#include <AutoRefVector.h>

namespace HAPI {

  /// \class HAPIHapticsDevice
  /// \brief Base class for all haptics devices.
  /// 
  /// The functions that should be implemented by all subclasses are:
  /// - bool initHapticsDevice()
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

    /// Class for holding a snapshot of current values of different properties
    /// of the haptics device.
    struct HAPI_API DeviceValues {
      DeviceValues():
        button_status( 0 ),
        user_data( NULL ) {}

      Vec3 force;      /// The force currently being rendered
      Vec3 torque;     /// The torque currently being rendered
      Vec3 position;   /// The position of the haptics device
      Vec3 velocity;   /// The velocity of the haptics device.
      Rotation orientation; /// The orientation of the haptics device.
      HAPIInt32 button_status; /// The status of the buttons.
      void *user_data;   /// Extra data that can be used be developers when
      /// supporting new device types.
    };

    typedef H3DUtil::AutoRefVector< HAPIHapticShape > HapticShapeVector;
    typedef H3DUtil::AutoRefVector< HAPIForceEffect > HapticEffectVector;

    /// Constructor.
    HAPIHapticsDevice() :
      thread( NULL ),
      device_state( UNINITIALIZED ),
      delete_thread( false ),
      time_in_last_loop( 0 ),
      setup_haptic_rendering_callback( true ) {
      setHapticsRenderer( NULL );
      haptic_rendering_callback_data = this;
    }
    
    /// Destructor. Stops haptics rendering and remove callback functions.
    virtual ~HAPIHapticsDevice() {}

    ////////////////////////////////////////////////////////////////////
    // Device handling functions.
    //

    /// Does all the initialization needed for the device before starting to
    /// use it.
    virtual ErrorCode initDevice();

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
    /// the device until the initDevice() function has been called again.
    inline virtual ErrorCode releaseDevice() {
      if( device_state == UNINITIALIZED ) {
        return NOT_INITIALIZED;
      }
      
      for( unsigned int i = 0; i < haptics_renderers.size(); i++ ) {
        if( haptics_renderers[i] ) {
          haptics_renderers[i]->releaseRenderer( this );
          haptics_renderers[i]->cleanUpStuff( this );
        }
      }

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
    /// \param objects The haptic shapes to render.
    /// \param layer The haptic layer to add the shape to.
    inline void addShape( HAPIHapticShape *shape, 
                          unsigned int layer = 0 ) {
      assureSize( layer );
      shape_lock.lock();
      tmp_shapes[layer].push_back( shape );
      shape_lock.unlock();
    }
    
    
    /// Set the HapticForceEffects to be rendered.
    /// \param objects The haptic shapes to render.
    /// \param layer The haptic layer to add the shape to.
    inline void setShapes( const HapticShapeVector &shapes, 
                           unsigned int layer = 0 ) {
      assureSize( layer );
      HapticShapeVector v = shapes;
      shape_lock.lock();
      v.swap( tmp_shapes[layer] );
      shape_lock.unlock();
    }

    /// Get the shapes currently used
    inline const HapticShapeVector &getShapes( unsigned int layer = 0 ) {
      assureSize( layer );
      return tmp_shapes[layer];
    }

    /// Remove a HAPIHapticShape from the shapes being rendered.
    /// \param layer The haptic layer to remove the shape from.
    inline void removeShape( HAPIHapticShape *shape,
                             unsigned int layer = 0 ) {
      assureSize( layer );
      shape_lock.lock();
      tmp_shapes[layer].erase( shape );
      shape_lock.unlock();
    }
    
    /// Add all shapes between [begin, end)
    /// \param layer The haptic layer to add shapes to
    template< class InputIterator > 
    inline void addShapes( InputIterator begin, InputIterator end,
                           unsigned int layer = 0 ) {
      assureSize( layer );
      shape_lock.lock();
      tmp_shapes[layer].insert( tmp_shapes[0].end(), begin, end );
      shape_lock.unlock();
    }

    /// Swap the vector of shapes currently being rendered with the
    /// given vector, replacing all shapes being rendered.
    /// \param layer The haptic layer to add shapes to
    inline void swapShapes( HapticShapeVector &shapes, 
                            unsigned int layer = 0 ) {
      assureSize( layer );
      shape_lock.lock();
      tmp_shapes[layer].swap( shapes );
      shape_lock.unlock();
    }

    /// Remove all HAPIHapticShape objects that are currently being rendered.
    /// \param layer The haptic layer to clear
    inline void clearShapes( unsigned int layer = 0 ) {
      assureSize( layer );
      shape_lock.lock();
      tmp_shapes[layer].clear();
      shape_lock.unlock();
    }

    /// Add a HapticForceEffect to be rendered.
    /// \param objects The haptic shapes to render.
    inline void addEffect( HAPIForceEffect *effect ) {
      force_effect_lock.lock();
      current_force_effects.push_back( effect );
      force_effect_lock.unlock();
    }

    /// Set the HapticForceEffects to be rendered.
    /// \param objects The haptic shapes to render.
    inline void setEffects( const HapticEffectVector &effects ) {
      force_effect_lock.lock();
      current_force_effects = effects;
      force_effect_lock.unlock();
    }

    /// Get the shapes currently used
    inline const HapticEffectVector &getEffects() {
      force_effect_lock.lock();
      return current_force_effects;
      force_effect_lock.unlock();
    }

    /// Remove a force effect so that it is not rendered any longer.
    inline void removeEffect( HAPIForceEffect *effect ) {
      force_effect_lock.lock();
      current_force_effects.erase( effect );
      force_effect_lock.unlock();
    }

    /// Add all effects between [begin, end)
    template< class InputIterator > 
    inline void addEffects( InputIterator begin, InputIterator end ) {
      force_effect_lock.lock();
      current_force_effects.insert( current_force_effects.end(), begin, end );
      force_effect_lock.unlock();
    }

    /// Swap the vector of effects currently being rendered with the
    /// given vector, replacing all effects being rendered.
    inline void swapEffects( HapticEffectVector &effects ) {
      force_effect_lock.lock();
      current_force_effects.swap( effects );
      force_effect_lock.unlock();
    }

    /// Remove all HAPIHapticShape objects that are currently being rendered.
    inline void clearEffects() {
      force_effect_lock.lock();
      current_force_effects.clear();
      force_effect_lock.unlock();
    }
    
    /// Transfer all current haptic objects to be rendered by the haptics
    /// renderer.
    virtual void transferObjects();

    //////////////////////////////////////////////////////////////
    // Functions for getting/settings device values
    //
    
    /// \brief Set the position calibraion matrix, i.e. the transform matrix
    /// from the local device coordinate space to HAPI coordinate space.
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
    /// specified for a specified layer.
    inline void setHapticsRenderer( HAPIHapticsRenderer *r, 
                                    unsigned int layer = 0 ) {


      
      if( haptics_renderers.size() < layer + 1 ) {
        renderer_change_lock.lock();
        haptics_renderers.resize( layer + 1, NULL );
        renderer_change_lock.unlock();
      }
      // TODO: synchronise with haptic thread in a way that does not
      // lock up openhaptics if openhaptics is used. Right now it is
      // not correctly synchronised or locked.
      if( device_state != UNINITIALIZED ) {
        if( haptics_renderers[layer] ) {
          haptics_renderers[layer]->releaseRenderer( this );
        }
        if( r )
          r->initRenderer( this );
      }
 
      renderer_change_lock.lock();
      // give ownership to temporary auto_ptr to get correct deletion of 
      // the renderer that is replaces since the auto_ptr_vector does not
      // take care of this.
      auto_ptr< HAPIHapticsRenderer > temp_rendr( haptics_renderers[ layer ] );
      haptics_renderers[ layer ] = r;
      renderer_change_lock.unlock();
    }

    /// Return the number of layers that have had shapes added to it.
    inline unsigned int nrLayers() {
      return haptics_renderers.size();
    }

    /// Get the currently used HAPIHapticsRenderer for a layer.
    inline HAPIHapticsRenderer *getHapticsRenderer( unsigned int layer = 0 ) {
      if( haptics_renderers.size() < layer + 1 ) return NULL;
      else return haptics_renderers[ layer ];
    }

    /// Get the error message from the latest error.
    inline const string &getLastErrorMsg() {
      return last_error_message;
    }

    /// Get the current actual update rate in loops per second
    inline unsigned int getHapticsRate() { return haptics_rate; }

    /// Get the time spent in the last haptics loop(in seconds)
    inline HAPITime getTimeSpentInLastLoop() {
      return time_in_last_loop;
    }

    /// Get the thread that is used to run this haptics device.
    inline H3DUtil::PeriodicThreadBase *getThread() {
      return thread;
    }

    // The following is part of the database of available haptics renderers.
    typedef HAPIHapticsDevice*( *CreateInstanceFunc)(); 

    template< class N >
    static HAPIHapticsDevice *newInstance() { return new N; };

    /// Class used to register a class to the registered file readers.
    struct HAPI_API HapticsDeviceRegistration{
    public:
      /// Constructor.
      HapticsDeviceRegistration( const string &_name,
                                   CreateInstanceFunc _create,
                                   list< string > _libs_to_support  ):
      name( _name ),
      create_func( _create ),
      libs_to_support( _libs_to_support )
      {
        
        if( !initialized ) {
          HAPIHapticsDevice::registered_devices.reset( 
            new list< HapticsDeviceRegistration > );
          initialized = true;
        }
        HAPIHapticsDevice::registerDevice( *this );
      }

      string name;
      CreateInstanceFunc create_func;
      list< string > libs_to_support;
    };
#ifdef __BORLANDC__
    friend struct HapticsDeviceRegistration;
#endif

    /// Register a haptics renderer to the database.
    /// \param name The name of the renderer
    /// \param create A function for creating an instance of that class.
    /// \param libs_to_support A list of strings with libraries that is needed
    /// to be supported by this device (dlls, so).
    static void registerDevice( const string &name,
                                  CreateInstanceFunc create,
                                  list< string > libs_to_support ) {
      registerDevice( HapticsDeviceRegistration( name, create, 
                                                 libs_to_support ) );
    }

    /// Register a haptics renderer that can then be returned by 
    /// getSupportedFileReader().
    static void registerDevice( const HapticsDeviceRegistration &fr ) {
      registered_devices->push_back( fr );
    }

        // Creating a new auto_ptr local for this node, because 
    // registrated_file_reader caused a memory leak and because
    // of the order of setting the static variables the autp_ptr's
    // constructor resets the auto_ptr to 0 eventhough the 
    // registrated_file_reader has been initilazed, and therefore
    // cause an error making it imposible to use the standard auto_ptr.
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

    static local_auto_ptr< list< HapticsDeviceRegistration > >
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
        output.torque = position_calibration_inverse.getRotationPart() * t;
        device_values_lock.unlock();
      }
    }

    /// Output that is sent to the haptics device to render.
    struct HAPI_API DeviceOutput {
      Vec3 force;  /// The force in Newtons
      Vec3 torque; /// The torque in Newtons/mm
      Matrix4 position_force_jacobian;
    };

    /// Updates the current_device_values member to contain
    /// current values.
    void updateDeviceValues( HAPITime dt );
    
    /// Calculates instant velocity using current_raw_device_values
    /// and the device values sent as input.
    inline void calculateVelocity( DeviceValues &dv,
                                   HAPITime dt ) {
      dv.velocity = ( dv.position - current_raw_device_values.position ) / dt;
    }

    /// Sends the data in the output member to be rendered at
    /// the haptics device.
    inline void sendOutput() {
      if( device_state == ENABLED ) {
        device_values_lock.lock();
        sendOutput( output, 0 );
        device_values_lock.unlock();
      }
    }

    /// Sets an error message that can be accessed later through
    /// getLastErrorMsg()
    inline void setErrorMsg( const string &err ) {
      last_error_message = err;
    }

    /// This function should be overriden by all subclasses of 
    /// HAPIHapticsDevice in order to fill in the given DeviceValues
    /// structure with current values of the devie.
    virtual void updateDeviceValues( DeviceValues &dv,
                                     HAPITime dt ) {
      dv.force = output.force;
      dv.torque = output.torque;
    }

    /// This function should be overridden by all subclasses of 
    /// HAPIHapticsDevice in order to send the values in the
    /// given DeviceOutput structure to the haptics device.
    virtual void sendOutput( DeviceOutput &dv,
                             HAPITime dt ) = 0;

    /// Initialize the haptics device.
    virtual bool initHapticsDevice() = 0;

    /// Release all resources allocated to the haptics device.
    virtual bool releaseHapticsDevice() = 0;

    /// The thread that this haptics device loop is run in.
    H3DUtil::PeriodicThreadBase *thread;

    // the force effects that are currently rendered in the realtime loop.
    // Should not be changed directly from the scenegraph loop but instead
    // use the renderEffects function to set the effects.
    HapticEffectVector current_force_effects;

    // the force effects that was current in the last scene graph loop.
    HapticEffectVector last_force_effects;

    // the shapes that are currently being rendered in the realtime loop.
    // One HapticEffectVector for each layer.
    vector< HapticShapeVector > current_shapes;

    // the values to send to the haptics device.
    DeviceOutput output;

    // lock for when changing the shapes to be rendered
    H3DUtil::MutexLock shape_lock;

    // lock for when updating device values/sending output
    H3DUtil::MutexLock device_values_lock;

    // lock for when changing haptics renderer
    H3DUtil::MutexLock renderer_change_lock;

    // lock for when using force effects
    H3DUtil::MutexLock force_effect_lock;

    //
    vector< HapticShapeVector > tmp_shapes ;

    /// The time at the beginning of the last rendering loop
    TimeStamp last_loop_time;
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
    string last_error_message;
    string device_name;

    /// Callback function to render force effects.
    static H3DUtil::PeriodicThread::CallbackCode hapticRenderingCallback( void *data );

    /// If true then set up hapticRenderingCallback as its own callback.
    /// If false the hapticRenderingCallback functions has to be called
    /// explicity from somewhere.
    bool setup_haptic_rendering_callback;

    /// Use this to send to hapticRenderingCallback when
    /// setup_haptic_rendering_callback is false. Needed to have
    /// anydevice call the callback correctly depending on
    /// setup_haptic_rendering_callback.
    void * haptic_rendering_callback_data;

    /// Callback function to transfer haptic objects to render to the 
    /// haptics loop.
    static H3DUtil::PeriodicThread::CallbackCode transferObjectsCallback( void *data );

    friend class AnyHapticsDevice;

    unsigned int nr_haptics_loops;
    unsigned int haptics_rate;
    TimeStamp last_hr_update;
    
  };
}

#endif
