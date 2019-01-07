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
/// \file ParsedFunction.cpp
/// \brief cpp file for ParseFunction.
///
//
//////////////////////////////////////////////////////////////////////////////

#include <HAPI/ParsedFunction.h> 

#ifdef HAVE_FPARSER
// H3DUtil include
#include <H3DUtil/Console.h>
// fparser includes
#include <fparser.hh>

using namespace HAPI;

ParsedFunction::ParsedFunction():
  have_valid_function( false ),
  fparser( new FunctionParser ) {
}

ParsedFunction::~ParsedFunction() {
  if( fparser ) {
    delete fparser;
    fparser = NULL;
  }
}

bool ParsedFunction::setFunctionString( const std::string &function,
                                        const std::string &evaluation_params ) {
  function_string = function;
  params_string = evaluation_params;
  int res = fparser->Parse( function_string, evaluation_params );
  if(res < 0) {
    have_valid_function = true;
    return true;
  }
 
  H3DUtil::Console(H3DUtil::LogLevel::Warning) << fparser->ErrorMsg() << std::endl;
  have_valid_function = false;
  return false;
}

/// Evaluate the function. 
/// input points to the input values to the function.
HAPIFloat ParsedFunction::evaluate( HAPIFloat *input ) {
  if( have_valid_function ) {
    HAPIFloat v = fparser->Eval( input );
    if( fparser->EvalError() == 0 ) {
      return v;
    } else {
      // we had an error evaluating (usually NaN). Function
      // classes are expected to use NaN and Inf but this is 
      // not possible here since Eval returns 0 on these
      // instances and we have no way identifying what the error
      // was. Always return NaN do get it right in most cases. 
      return std::numeric_limits<HAPIFloat>::quiet_NaN();
    }
  }
  else return 0;
}

#endif
