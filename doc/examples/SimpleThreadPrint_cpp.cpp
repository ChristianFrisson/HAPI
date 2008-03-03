#include <H3DUtil/Threads.h>

// To be able to use std functions.
#include <iostream>
#include <string>
#include <vector>
using namespace std;

// The function to run in a separate thread.
void * printFunction(void *argument) {
  string * to_print = (string *)argument;
  while( true )
    cerr << *to_print << endl;
  return 0;
}

// Main function.
int main(int argc, char* argv[]) {
  // Creating the thread. The thread will start printing to the console.
  // The second argument contains what should be printed. It is done
  // in this way to show how to send arguments to the function.
  string to_print = "SimpleThread";
  H3DUtil::SimpleThread a_simple_thread( printFunction, 
                                        (void *)(&to_print) );
  // while at the same time this is printed.
  cerr << "Press ENTER to exit" << endl;

  string temp_string;
  getline( cin, temp_string );
}
