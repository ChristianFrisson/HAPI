//////////////////////////////////////////////////////////////////////////////
//    Copyright 2008-2019, SenseGraphics AB
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
/// \file SimpleThreadPrint.cpp
/// \brief CPP file which provides example code to show H3DUtil::PeriodicThread
/// and its callback functionality.
///
//
//////////////////////////////////////////////////////////////////////////////
#include <H3DUtil/Threads.h>

// To be able to use std functions.
#include <iostream>
#include <string>
#include <vector>
using namespace std;

// The function we will use for our asynchronousCallback.
H3DUtil::PeriodicThreadBase::CallbackCode printAsynchronous(
                                              void *argument ) {
  string * to_print = (string *)argument;
  cerr << *to_print << endl;
  return H3DUtil::PeriodicThreadBase::CALLBACK_CONTINUE;
}

// The function we will use for our synchronousCallback. It will change
// the argument to the asynchronousCallback.
H3DUtil::PeriodicThreadBase::CallbackCode printSynchronous(
                                             void *argument ) {
  string * to_print = (string *)argument;
  cerr << "synchronousCallback" << endl;
  *to_print = "asynchCallback";
  return H3DUtil::PeriodicThreadBase::CALLBACK_DONE;
}

// Main function.
int main(int argc, char* argv[]) {
  // Creating the PeriodicThread.
  H3DUtil::PeriodicThread periodic_thread(
    H3DUtil::ThreadBase::NORMAL_PRIORITY, 3 );
  cerr << "Press ENTER to add an asynchronousCallback "
       << "that will be called each loop in the thread. "
       << "The number of loops per second is 3." << endl;

  string temp_string;
  getline( cin, temp_string );

  string to_print = "asynchronousCallback";
  int asynch_callback_handle =
    periodic_thread.asynchronousCallback( printAsynchronous,
                                          (void *)(&to_print) );

  cerr << "This could be printed before "
       << "\"asynchronousCallback\" prints. Press ENTER to add"
       << " a synchronousCallback." << endl;
  getline( cin, temp_string );

  periodic_thread.synchronousCallback( printSynchronous,
                                       (void *)(&to_print) );

  cerr << "This line will never be printed until after"
       << " synchronousCallback has been printed. "
       << "Press ENTER to remove the asynchronousCallback." << endl;
  getline( cin, temp_string );

  periodic_thread.removeAsynchronousCallback( asynch_callback_handle );

  cerr << "Press ENTER to exit" << endl;
  getline( cin, temp_string );
}

