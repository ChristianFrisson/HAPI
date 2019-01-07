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
/// \file HAPI/DeviceLog.h
/// \brief header file for DeviceLog class.
//
//
//////////////////////////////////////////////////////////////////////////////
#ifndef __DEVICELOG_H__
#define __DEVICELOG_H__

#include <H3DUtil/Threads.h> 
#include <HAPI/HAPIForceEffect.h> 
#include <HAPI/HAPIHapticsDevice.h>
#include <fstream>

namespace HAPI {

  /// \ingroup ForceEffects
  /// \class DeviceLog
  /// \brief Logs haptics device information at frequency given as
  /// argument to constructor. The data could either be saved in a binary file
  /// or in a ascii text file.
  ///
  /// The first line in a log file in ascii format will always contains a
  /// couple of strings separated by whitespace specifying what haptics 
  /// device data is stored in the file. The rest of the file will consist 
  /// of lines with values for the data. Each value is separated by space.
  /// An example of an ASCII file written down with log frequency 100Hz
  /// follows here:
  ///
  /// TIME POSITION ORIENTATION
  /// 0 0.1 0.2 0.3 0 0 1 0.5
  /// 0.01 0.1 0.22 0.3 0 0 1 0.51
  /// 0.02 0.1 0.22 0.3 0 0 1 0.51
  ///
  /// A binary file will contain strings specifying what haptics device data
  /// is stored in the file. After the last string is read the next section
  /// will be a null character ('\\0'). This fact should be used to know when
  /// all the strings are read. After all the strings are read the data will
  /// come in the order specified by the strings. Repeated until end of file.
  ///
  /// Here follows example C++ code showing how to read from a log file in
  /// binary format stored by this class and print the output to the console:
  ///
  /// \code
  /// // Create file stream and open file.
  /// ifstream file_stream;
  /// file_stream.open( "filename", ios::in | ios::binary );
  /// if( file_stream.is_open() ) {
  ///   vector < string > what_is_logged;
  ///   // Read the first strings.
  ///   while( !file_stream.eof() && file_stream.peek() != '\0' ) {
  ///     string tmp;
  ///     readH3DType( file_stream, tmp );
  ///     what_is_logged.push_back( tmp );
  ///   }
  ///   // Read the next character '\0' in order to put the stream pointer
  ///   // in the correct position.
  ///   char tmp;
  ///   file_stream.read( &tmp, sizeof( char ) );
  ///   // Print to console.
  ///   for( unsigned int i = 0; i < what_is_logged.size(); ++i )
  ///     cerr << what_is_logged[i] << " ";
  ///   cerr << endl;
  ///   // Read until end of file and print values to console.
  ///   while( !file_stream.eof() ) {
  ///     for( unsigned int i = 0; i < what_is_logged.size(); ++i ) {
  ///       if( what_is_logged[i] == "TIME" ) {
  ///         HAPI::HAPITime tmp;
  ///         readH3DType( file_stream, tmp );
  ///         cerr << tmp;
  ///       } else if( what_is_logged[i] == "POSITION" ||
  ///                  what_is_logged[i] == "VELOCITY" ||
  ///                  what_is_logged[i] == "FORCE" ||
  ///                  what_is_logged[i] == "TORQUE" ||
  ///                  what_is_logged[i] == "RAW_POSITION" ||
  ///                  what_is_logged[i] == "RAW_VELOCITY" ||
  ///                  what_is_logged[i] == "RAW_FORCE" ||
  ///                  what_is_logged[i] == "RAW_TORQUE" ) {
  ///         HAPI::Vec3 tmp;
  ///         readH3DType( file_stream, tmp );
  ///         cerr << tmp;
  ///       } else if( what_is_logged[i] == "ORIENTATION" ||
  ///                  what_is_logged[i] == "RAW_ORIENTATION" ) {
  ///         HAPI::Rotation tmp;
  ///         readH3DType( file_stream, tmp );
  ///         cerr << tmp;
  ///       } else if( what_is_logged[i] == "BUTTONS" ) {
  ///         HAPI::HAPIInt32 tmp;
  ///         readH3DType( file_stream, tmp );
  ///         cerr << tmp;
  ///       }
  ///       cerr << " ";
  ///     }
  ///     cerr << endl;
  ///   }
  /// }
  /// file_stream.close();
  /// \endcode
  class HAPI_API DeviceLog: public HAPIForceEffect {
  public:
    /// Specifies different properties of the haptics device
    /// that can be logged.
    typedef enum {
      TIME = 0,
      POSITION = 1,
      ORIENTATION = 2,
      VELOCITY = 3,
      FORCE = 4,
      TORQUE = 5,
      BUTTONS = 6,
      RAW_POSITION = 7,
      RAW_ORIENTATION = 8,
      RAW_VELOCITY = 9,
      RAW_FORCE = 10,
      RAW_TORQUE = 11,
      TIMESTAMP = 12
    } LogType;

    typedef std::vector< LogType > LogTypeVector;

    /// Constructor
    /// \param _log_file The name/url of the file to log to.
    /// \param _log_type Contains information about which parameters
    ///                  of the haptics device should be logged.
    /// \param _freq The frequency of logging.
    /// \param _binary If true the logging will be done to a binary file.
    ///                Otherwise it will be done to an ASCII-text file.
    /// \param _use_ref_count_lock If true, the force effect will have lock 
    ///                            protected ref counting to guarantee its thread safety
    DeviceLog( const std::string &_log_file, const LogTypeVector &_log_type,
               int _freq = 100, bool _binary = false, bool _use_ref_count_lock = false );

    /// The force of the EffectOutput will zero. Only logging to file
    /// will be done.
    /// \param input Contains useful information, see EffectInput struct.
    virtual EffectOutput calculateForces( const EffectInput &input );

    /// Stop logging and flush and close the log file
    virtual void close ();

    /// Get the last device values which were logged
    /// 
    /// This may be used instead of device->getDeviceValues() for cases where
    /// we need the live interaction to match the recording/playback exactly
    ///
    /// \note Must be called from the same thread that is used for logging
    const HAPIHapticsDevice::DeviceValues& getLastLoggedDeviceValues();

    /// Get the last raw device values which were logged
    /// 
    /// This may be used instead of device->getRawDeviceValues() for cases where
    /// we need the live interaction to match the recording/playback exactly 
    ///
    /// \note Must be called from the same thread that is used for logging
    const HAPIHapticsDevice::DeviceValues& getLastLoggedRawDeviceValues();

  protected:
    // Contains certain times needed in order to know when to log.
    HAPITime last_time, start_time;
    HAPITime time_diff;
    // The file stream to which writing is done.
    std::ofstream log_file;
    // Contains information about which parameters of the haptics device
    // should be logged.
    LogTypeVector log_type;
    bool binary;
    
    // Internal helper functions.
    void writeLog( const EffectInput &input, HAPITime log_time );
    void writeHeader( const EffectInput &input );

    // Virtual functions to write log data. Sub-classes should override
    // these in order to log additional values.
    virtual void writeLogRow ( const EffectInput &input, HAPITime log_time );
    virtual void writeHeaderRow ( const EffectInput &input );

    /// A callback executed in the haptic thread when the node is destroyed
    /// in order to force the HAPI loggers to flush and close their logging files
    static H3DUtil::PeriodicThread::CallbackCode closeCallback ( void* data );

    /// The last device values which were written to the log
    HAPIHapticsDevice::DeviceValues last_logged_device_values;

    /// The last raw device values which were written to the log
    HAPIHapticsDevice::DeviceValues last_logged_raw_device_values;
  };
}

#endif

