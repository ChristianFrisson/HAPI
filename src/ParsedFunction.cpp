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
/// \file ParsedFunction.cpp
/// \brief cpp file for ParseFunction.
///
//
//////////////////////////////////////////////////////////////////////////////

#include <HAPI/ParsedFunction.h> 

#ifdef HAVE_FPARSER
// H3DUtil include
#include <Console.h>

using namespace HAPI;

ParsedFunction::ParsedFunction():
  have_valid_function( false ),
  fparser( new FunctionParser ) {
}

bool ParsedFunction::setFunctionString( const string &function,
                                        const string &evaluation_params ) {
  function_string = function;
  params_string = evaluation_params;
  int res = fparser->Parse( function_string, evaluation_params );
  if(res < 0) {
    have_valid_function = true;
    return true;
  }
 
  H3DUtil::Console(3) << fparser->ErrorMsg() << endl;
  have_valid_function = false;
  return false;
}

/// Evaluate the function. 
/// input points to the input values to the function.
HAPIFloat ParsedFunction::evaluate( HAPIFloat *input ) {
  if( have_valid_function )
    return fparser->Eval( input );
  else return 0;
}

#endif
