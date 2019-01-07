//////////////////////////////////////////////////////////////////////////////
//    Copyright 2011-2019, SenseGraphics AB
//
//    Any use, or distribution, of this file without permission from the
//    copyright holders is strictly prohibited. Please contact SenseGraphics,
//    www.sensegraphics.com, for more information.
//
//
/// \file ClutchedHapticsDevice.h
/// \brief Header file for ClutchedHapticsDevice.
///
//
//////////////////////////////////////////////////////////////////////////////

#ifndef __CLUTCHEDHAPTICSDEVICE_H__
#define __CLUTCHEDHAPTICSDEVICE_H__

#include <HAPI/HAPIHapticsDevice.h>
#include <H3DUtil/Threads.h>

namespace HAPI {

  /// \ingroup HapticsDevices
  /// \class ClutchedHapticsDevice
  /// \brief This class represents a wrapper which adds clutching functionality
  class HAPI_API ClutchedHapticsDevice: public HAPIHapticsDevice {
  public:
    /// Constructor.
    ClutchedHapticsDevice() : hd ( NULL ), clutchEnabled ( false ) {}

    /// Destructor.
    virtual ~ClutchedHapticsDevice() {}

    /// Returns a pointer to the device that is actually used.
    inline HAPIHapticsDevice *getActualHapticsDevice() { 
      return hd;
    }

    /// Sets the actual haptics device which is being wrapped.
    inline void setActualHapticsDevice( HAPIHapticsDevice* _hapticsDevice ) { 
      hd= _hapticsDevice;
    }

    /// Enable the device. Positions can be read and force can be sent.
    inline virtual ErrorCode enableDevice() {
      ErrorCode e = HAPIHapticsDevice::enableDevice();
      if(hd ) hd->enableDevice();
      return e;
    }

    /// Disable the device. 
    inline virtual ErrorCode disableDevice() {
      ErrorCode e = HAPIHapticsDevice::disableDevice();
      if(hd ) hd->disableDevice();
      return e;
    }

    /// Enable or disable the clutch
    void enableClutch ( bool enable );

    /// Returns the unclutched(real) values from the wrapped haptics device.
    DeviceValues getUnclutchedDeviceValues () {
      DeviceValues dv;
      clutchedDeviceValuesLock.lock();
      dv= unclutchedDeviceValues;
      clutchedDeviceValuesLock.unlock();
      return dv;
    }

    /// Resets the clutch orientation/position.
    void resetClutch ( bool _resetOrientation= true, bool _resetPosition= true ) {
      clutchLock.lock();
      if ( _resetOrientation ) clutchOrientationOffset= Rotation();
      if ( _resetPosition ) clutchPositionOffset= Vec3();
      clutchLock.unlock();
    }

  protected:
    /// Implementation of updateDeviceValues using the contained device
    /// to get the values.
    /// \param dv Contains values that should be updated.
    /// \param dt Time since last call to this function.
    virtual void updateDeviceValues( DeviceValues &dv, HAPITime dt ) {
      if( hd ) {
        hd->updateDeviceValues( dv, dt );
        // Needed to correctly calculate device velocity for devices that
        // base this on current_raw_device_values, for example
        // FalconHapticsDevice
        hd->current_raw_device_values = last_raw_device_values;

        clutchedDeviceValuesLock.lock();
        unclutchedDeviceValues= dv;
        clutchedDeviceValuesLock.unlock();

        clutchLock.lock();
        if ( clutchEnabled ) {
          dv.orientation= startClutchOrientation;
          dv.position= startClutchPosition;
        } else {
          dv.orientation= dv.orientation*-clutchOrientationOffset;
          dv.position= dv.position+clutchPositionOffset;
        }
        clutchLock.unlock();
      }
    }

    /// Implementation of sendOutput, calling sendOutput of the contained
    /// device.
    /// \param dv Contains force values to send to the haptics device.
    /// \param dt Time since last call to this function.
    virtual void sendOutput( DeviceOutput &dv,
                             HAPITime dt ) {
      if( hd ) {
        hd->output.force = output.force;
        hd->output.torque = output.torque;
        hd->output.dof7_force = output.dof7_force;
        hd->sendOutput( dv, dt );
      }
    }

    /// Calls initHapticsDevice of the contained device.
    virtual bool initHapticsDevice( int _thread_frequency = 1000 );

    /// Releases all resources allocated in initHapticsDevice. 
    virtual bool releaseHapticsDevice() {
      if( hd ) {
        bool b = hd->releaseHapticsDevice();
        hd= NULL;
        thread = NULL;
        return b;
      }
      return true;
    }


    /// The haptics device actually used.
    HAPIHapticsDevice* hd;

    H3DUtil::MutexLock clutchedDeviceValuesLock;

    // Real raw device position and orientation before clutch is accounted for
    DeviceValues unclutchedDeviceValues;

    H3DUtil::MutexLock clutchLock;

    bool clutchEnabled;

    Rotation startClutchOrientation;
    Rotation clutchOrientationOffset;

    Vec3 startClutchPosition;
    Vec3 clutchPositionOffset;
  };
}

#endif
