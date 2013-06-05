//////////////////////////////////////////////////////////////////////////////
//    Copyright 2004-2013, SenseGraphics AB
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
/// \file PlaybackHapticsDevice.h
/// \brief Header file for PlaybackHapticsDevice.
///
//
//////////////////////////////////////////////////////////////////////////////

#ifndef __HAPI_PLAYBACKHAPTICSDEVICE_H__
#define __HAPI_PLAYBACKHAPTICSDEVICE_H__

#include <HAPI/HAPIHapticsDevice.h>
#include <fstream>

namespace HAPI {

  class HAPI_API PlaybackHapticsDevice: public HAPIHapticsDevice {
  public:

    /// Constructor.
    PlaybackHapticsDevice () :
      data_start_pos ( 0 ),
      playback_time ( 0 ),
      playing ( false ),
      playback_start_time ( 0 ),
      seek_to_time ( 0 ),
      playback_speed ( 1 ),
      binary ( false ) {
    }

    virtual void updateDeviceValues( DeviceValues &dv, HAPI::HAPITime dt );

    /// Output is ignored on a fake haptics device.
    virtual void sendOutput( DeviceOutput &dv,
            HAPI::HAPITime dt ) {}

    /// Implementation of initHapticsDevice.
    virtual bool initHapticsDevice( int _thread_frequency = 1000 ) {
      return true;
    }

    /// Releases all resources allocated in initHapticsDevice. 
    virtual bool releaseHapticsDevice() { return true; }

    /// Specify the filename to use to read playback data from.
    /// 
    /// The file format is the same format that is produced by the DeviceLog
    /// force effect and is described in more detail there. For synchronized 
    /// playback the file should conatian the TIME column. The following columns
    /// are played back, if present: RAW_POSITION, RAW_ORIENTATION, RAW_VELOCITY,
    /// BUTTONS. All other columns are ignored.
    ///
    /// \param _filename The filename of the file containing the playback data
    /// \param _binary   If true then the file is considered to contain binary data
    ///                  otherwise it is considered to contain text data
    ///
    /// \return True if the file was opened successfully for reading
    bool loadRecording ( const std::string& _filename, bool _binary= false );

    void closeRecording ();

    /// Start playing back the loaded recording from the current playback position
    void startPlayback ();

    /// Stop playing back the recording. 
    ///
    /// All device outputs are frozen at there last values and
    /// playback can be resumed by calling startPlayback()
    void stopPlayback ();

    /// Move playback position to the specified point in time. 
    ///
    /// This function only takes effect the next time playback is started.
    ///
    /// \param _time The time in the recording to move to
    void seekToTime ( HAPITime _time );

    /// Set the speed of the playback.
    ///
    /// This function only takes effect the next time playback is started.
    ///
    /// \param _speed The desired speed of the playback, where 1 is the 
    ///               original recording speed.
    void setPlaybackSpeed ( HAPIFloat _speed );

    /// Returns the time of the last event that was played back
    HAPITime getPlaybackTime ();

    /// Returns true if currently playing back events. False if idle, or
    /// the playback has completed.
    bool isPlaying ();

  protected:
    typedef std::vector < std::string > StringList;

    /// Read the next entry in the recording
    ///
    /// \param[out] _dv   The device values read from the recording
    /// \param[out] _time The time of the device values in the recording
    ///
    bool getPlaybackValuesNext ( HAPI::HAPIHapticsDevice::DeviceValues& _dv, HAPI::HAPITime& _time );

    /// Read the next entry in the recording that corresponds to the specified time
    ///
    /// Entries are read from the recording until the time stamp of an entry is >= _time
    ///
    /// \param[out] _dv   The device values read from the recording
    /// \param[in] _time  The time to search forward for
    ///
    bool getPlaybackValuesAtTime ( HAPI::HAPIHapticsDevice::DeviceValues& _dv, HAPI::HAPITime _time );

    bool getPlaybackValuesNextBinary ( HAPI::HAPIHapticsDevice::DeviceValues& _dv, HAPI::HAPITime& _time );

    bool getPlaybackValuesNextText ( HAPI::HAPIHapticsDevice::DeviceValues& _dv, HAPI::HAPITime& _time );

    StringList readColumnNamesBinary ();

    StringList readColumnNamesText ();

    std::ifstream playback_file;

    std::streampos data_start_pos;

    StringList field_names;

    HAPITime playback_time;

    HAPIHapticsDevice::DeviceValues playback_device_values;

    bool playing;

    HAPITime playback_start_time;

    HAPITime seek_to_time;

    HAPITime playback_speed;

    bool binary;

    /// A mutex lock used to access playback data
    H3DUtil::MutexLock playback_lock;
  };
}

#endif
