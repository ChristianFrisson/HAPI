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

  /// \ingroup HapticsDevices
  /// \class PlaybackHapticsDevice
  /// \brief A fake haptics device that plays back values recorded
  ///        previously using a DeviceLog.
  /// 
  /// This haptics device plays back device values from a file. 
  /// The format of the file is the same as that produced
  /// by the DeviceLog force effect. Both binary and text formats are 
  /// supported.
  ///
  /// Any combination of columns are supported, however a TIME
  /// column is required in order to play back the values at the original
  /// speed.
  ///
  /// The following columns are used to define the device's raw values:
  /// RAW_POSITION, RAW_ORIENTATION, RAW_VELOCITY, BUTTONS. Any other 
  /// columns are ignored. The calibrated device values are calculated
  /// based on the raw values in the usual way and the recorded calibrated
  /// values (if present) are ignored.
  ///
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
      binary ( false ),
      got_first_value ( false ) {
    }

    /// Populates the DeviceValues structure with values from the
    /// recording currently being played back.
    ///
    /// \param dv Contains values that should be updated.
    /// \param dt Time since last call to this function.
    virtual void updateDeviceValues( DeviceValues &dv, HAPI::HAPITime dt );

    /// Output is ignored on a fake haptics device.
    virtual void sendOutput( DeviceOutput &dv,
            HAPI::HAPITime dt ) {}

    /// Dummy implementation of initHapticsDevice.
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

    /// Close and release the file previously opened using loadRecording()
    ///
    /// The file is closed automatically when the device is destroyed or a
    /// new recording is opened.
    ///
    /// Calling this function will stop playback, if active.
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

    /// Set default values to use when a value is not defined in the recording
    /// or is not in the list of fields that should be played back.
    ///
    /// Also see: addDataField() and clearDataFields()
    ///
    void setDefaultDeviceValues ( const HAPIHapticsDevice::DeviceValues& _dv );

    /// Add a column field name to the list of columns that should be played back
    ///
    /// If the list is empty then all available columns are played back. If a column
    /// is not played back, then the default value specified using setDefaultDeviceValues()
    /// is used instead.
    ///
    void addDataField ( const std::string& _name );

    /// Clear the list of columns that should be played back.
    ///
    /// If the list is empty then all available columns are played back. 
    ///
    void clearDataFields ();

    /// Read the next entry in the recording
    ///
    /// \param[out] _dv   The device values read from the recording
    /// \param[out] _time The time of the device values in the recording
    /// \param[out] _timestamp The absolute timestamp of the device values in the recording
    ///
    bool getPlaybackValuesNext ( HAPIHapticsDevice::DeviceValues& _dv, HAPITime& _time, HAPITime& _timestamp );

  protected:
    typedef std::vector < std::string > StringList;

    /// Read the next entry in the recording that corresponds to the specified time
    ///
    /// Entries are read from the recording until the time stamp of an entry is >= _time
    ///
    /// \param[out] _dv   The device values read from the recording
    /// \param[in] _time  The time to search forward for
    ///
    bool getPlaybackValuesAtTime ( HAPI::HAPIHapticsDevice::DeviceValues& _dv, HAPI::HAPITime _time );

    /// Implementation of getPlaybackValuesNext() for binary file types
    bool getPlaybackValuesNextBinary ( HAPI::HAPIHapticsDevice::DeviceValues& _dv, HAPI::HAPITime& _time, HAPITime& _timestamp );

    /// Implementation of getPlaybackValuesNext() for text file types
    bool getPlaybackValuesNextText ( HAPI::HAPIHapticsDevice::DeviceValues& _dv, HAPI::HAPITime& _time, HAPITime& _timestamp );

    /// Virtual function to read field values from binary file
    ///
    /// Subclasses may override this to read additional non-standard field values
    virtual void readFieldsValuesBinary ( HAPIHapticsDevice::DeviceValues& _dv, HAPITime& _time, HAPITime& _timestamp );

    /// Virtual function to read field values from text file
    ///
    /// Subclasses may override this to read additional non-standard field values
    virtual void readFieldsValuesText ( HAPIHapticsDevice::DeviceValues& _dv, HAPITime& _time, HAPITime& _timestamp );

    /// Read and returns a list of column headings from the recording file, when file is 
    /// in binary format.
    StringList readColumnNamesBinary ();

    /// Read and returns a list of column headings from the recording file, when file is 
    /// in text format.
    StringList readColumnNamesText ();

    /// The input file stream to playback data from
    std::ifstream playback_file;

    /// The stream position of the first recording data, after the file header
    std::streampos data_start_pos;

    /// List of column headers for columns present in the recording file
    StringList field_names;

    /// The recording time of the last event played back
    HAPITime playback_time;

    /// The last valid device values to have been played back
    HAPIHapticsDevice::DeviceValues playback_device_values;

    /// True when playback is active
    bool playing;

    /// The global system time at which playback started
    ///
    /// Note: This may be adjusted in order to account for pausing of the playback
    ///
    HAPITime playback_start_time;

    /// Recording time to move the playback to
    ///
    /// Will be reset to -1 after recording has been moved to this time which
    /// will occur next time playback is started.
    HAPITime seek_to_time;

    /// A factor by which to scale the playback speed
    HAPITime playback_speed;

    /// If true, then the recording is assumed to be in binary format, otherwise text
    bool binary;

    /// False until the first device values are read from the recording
    bool got_first_value;

    /// A mutex lock used to access playback data
    H3DUtil::MutexLock playback_lock;

    /// The list of columns that should be played back.
    ///
    /// If the list is empty then all available columns are played back. 
    StringList use_field_names;
  };
}

#endif
