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
  H3DUtil::PeriodicThread periodic_thread( DEFAULT_THREAD_PRIORITY, 3 );
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
