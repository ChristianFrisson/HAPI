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
/// \file Exception.h
/// \brief Base class for all H3D Exceptions
///
//
//
//////////////////////////////////////////////////////////////////////////////
#ifndef __EXCEPTION_H__
#define __EXCEPTION_H__

#include <string>
#include <iostream>
#include <HAPI.h>
using namespace std;

namespace H3D {
  namespace Exception {
    
    /// The base class for all exceptions
    class HAPI_API H3DException {
    public:
      
      /// Constructor
      /// \param _message optional message associtiated with the exception.
      ///
      H3DException( const string &_message = "" ) : message( _message ) {}

      virtual ~H3DException() {};
      /// Function for printing the exception to a stream.
      /// \param os the stream to print to.
      ///   
      virtual void print( ostream &os ) const  {
        os << "H3DException: " << message;
      }

      /// The message assoiciated with this exception
      string message;
    };

    /// This is the base class for exceptions where the location in the 
    /// source code is included. All subclasses must define the virtual function
    /// className(<>) that returns the name of the exception as a string
    /// in order to get proper printouts of the exceptions. To make
    /// instantiations of this class simpler the #H3D_API_EXCEPTION
    /// macro can be used to create the class. E.g.
    ///
    /// H3D_API_EXCEPTION( ParseError );
    ///
    /// will expand to define a subclass named ParseError.
    ///
    class HAPI_API H3DAPIException : public H3DException {
    public:
      /// Constructor. The macros #H3D_FULL_LOCATION, #H3D_FILE_LOCATION and
      /// #H3D_FUNCTION_LOCATION can be given as an argument to the constructor
      /// to specify the function, filename and linenumber instead of using the
      /// __FUNCTION__,__FILE__ and __LINE__ macros directly, i.e.
      /// \code 
      /// throw APIException( message, H3D_FULL_LOCATION )
      /// \endcode
      ///
      /// \param _message the message associated with the exception.
      /// \param _function the function in which the exception occured.
      /// \param _filename the file in which the exception occured. If 
      /// specified the macro __FILE__ should be used.
      /// \param _linenumber the line number in which the exception occured.
      /// If specified the macro __LINE__ should be used.
      ///
      H3DAPIException( const string &_message = "", 
                       const string &_function = "",
                       const string &_filename = "",
                       const int    &_linenumber = -1 ) : 
        H3DException( _message ),
        function(_function), filename(_filename), linenumber(_linenumber ) {}
  

      /// Virtual function that returns the name of the exception class and
      /// must be defined by all subclasses. The name is used to get a proper
      /// printout of the exception.
      /// \return the name of the class that is being defined
      ///
      virtual string className() const {
        return "H3DAPIException";
      }

      /// Function for printing the exception to a stream.
      /// \param os the stream to print to.
      /// 
      virtual void print( ostream &os ) const;
      
      /// The function in which the exception occured.
      string function;

      /// The name of the file in which the exception occured.
      string filename;

      /// The line number in which the exception occured.
      int linenumber;
    };

    /// A virtual template class for instantiating exceptions based 
    /// on a value. All subclasses must define the virtual function
    /// className(<>) that returns the name of the exception as a string
    /// in order to get proper printouts of the exceptions. To make
    /// instantiations of this class simpler the #H3D_VALUE_EXCEPTION
    /// macro can be used to create the class. E.g.
    ///
    /// H3D_VALUE_EXCEPTION( int, InvalidIdentifier );
    /// 
    /// will create an exception class called InvalidIdentifier with an
    /// integer as value.
    ///
    /// \param ValueType the type of the value that caused the exception.
    ///
    template < class ValueType >
    class ValueException : public H3DAPIException {
    public:
      /// Constructor.  The macros #H3D_FULL_LOCATION, #H3D_FILE_LOCATION and
      /// #H3D_FUNCTION_LOCATION can be given as an argument to the constructor
      /// to specify the function, filename and linenumber instead of using the
      /// __FUNCTION__,__FILE__ and __LINE__ macros directly
      /// 
      /// \param _value the value that caused the exception.
      /// \param _message the message associated with the exception.
      /// \param _function the function in which the exception occured.
      /// \param _filename the file in which the exception occured.
      /// \param _linenumber the line number in which the exception occured.
      ///
      ValueException( ValueType _value,
                      const string &_message  = "",
                      const string &_function = "",
                      const string &_filename = "",
                      const int &_linenumber  =  -1 ) : 
        H3DAPIException( _message, _function, _filename, _linenumber ),
        value( _value ) {}


      /// Virtual function that returns the name of the exception class and
      /// must be defined by all subclasses. The name is used to get a proper
      /// printout of the exception.
      /// \return the name of the class that is being defined
      ///
      virtual string className() const {
        return "ValueException";
      }

      /// Function for printing the exception to a stream.
      /// \param os the stream to print to.
      /// 
      virtual void print( ostream &os ) const {
        os << className() << ": " << value << ".";
        if( message.length() ) {
          os << " " << message;
        }
        if( function.length() || filename.length() || linenumber != -1 ) {
          os << " (";
          if( function.length() ) os << "" << function;
          if( filename.length() ) os << " in " << filename;
          if( linenumber != -1 ) os << " line " << linenumber;
          os << ")";
        }
      }
      
      /// The value that caused the exception.
      ValueType value;
    };
    
    /// This is special exception that can be thrown in order to quit the API. 
    /// It can be caught by the main() function to do final cleanup before
    /// termination. 
    class QuitAPI: public H3DException {};
   
    ///
    /// The << operator calls the print() function of the H3DException.
    /// \param os an ostream
    /// \param e the H3DException to put into the stream.
    /// 
    inline ostream &operator<<( ostream& os, const H3DException &e ) {
      e.print( os );
      return os;
    }
 
  }

  /// Macro for use with H3D::Exception::H3DAPIException constructor. Are to be 
  /// given to the constructor in place for the arguments function, filename 
  /// and linenumber. The exception will then include full info about the three 
  /// above mentioned arguments.
#ifdef __BORLANDC__
#define H3D_FULL_LOCATION __FUNC__, __FILE__, __LINE__
#else
#define H3D_FULL_LOCATION __FUNCTION__, __FILE__, __LINE__
#endif
  /// Macro for use with H3D::Exception::H3DAPIException constructor. Are to be
  /// given to the constructor in place for the arguments function, filename
  /// and linenumber. The exception will then include info about file name and
  /// line number of the exception.
#define H3D_FILE_LOCATION "", __FILE__, __LINE__

  /// Macro for use with H3D::Exception::H3DAPIException constructor. Are to be
  /// given to the constructor in place for the arguments function, filename
  /// and linenumber. The exception will then include info about the function
  /// in which the exception occured.
#ifdef __BORLANDC__
#define H3D_FUNCTION_LOCATION __FUNC__
#else
#define H3D_FUNCTION_LOCATION __FUNCTION__
#endif

  /// Macro for easy creation of a H3D::Exception::ValueException class.
  /// \param value_type the type of the value
  /// \param name the name of the class to define
  ///
#define H3D_VALUE_EXCEPTION( value_type, name )                             \
  class name : public H3D::Exception::ValueException< value_type > {        \
  public:                                                                   \
    name( value_type _value,                                                \
          const string &_message = "",                                      \
          const string &_function = "",                                     \
          const string &_filename = "",                                     \
          const int &_linenumber = -1 ) :                                   \
      H3D::Exception::ValueException< value_type >( _value, _message,       \
    _function,                                                              \
    _filename, _linenumber) {}                                              \
  protected:                                                                \
    string className() const { return #name; }                         \
}

  /// Macro for easy creation of a H3D::Exception::H3DAPIException subclass.
  /// \param name the name of the class to define
  ///
#define H3D_API_EXCEPTION( name )                                           \
  class name : public H3D::Exception::H3DAPIException {                     \
  public:                                                                   \
    name( const string &_message = "",                                      \
          const string &_function = "",                                     \
          const string &_filename = "",                                     \
          const int &_linenumber = -1 ) :                                   \
      H3D::Exception::H3DAPIException( _message, _function,                 \
                _filename, _linenumber) {}                                  \
  protected:                                                                \
    string className() const { return #name; }                         \
}

}

#endif
