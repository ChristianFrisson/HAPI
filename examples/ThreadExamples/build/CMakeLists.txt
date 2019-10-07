cmake_minimum_required( VERSION 2.8.7 )
cmake_policy( VERSION 2.8.7 )

# This will set up three projects. These three projects are examples
# from the manual.
project( ThreadExamples )

set( CMAKE_MODULE_PATH "${ThreadExamples_SOURCE_DIR}/../../../build/modules" )

include( H3DCommonFunctions )
handleCommonCacheVar( CMAKE_INSTALL_PREFIX ${ThreadExamples_SOURCE_DIR}/../.. )
setupRPathForLib()
enableCpp11( FAIL_MESSAGE "Enabling C++11 support. This compiler lacks such support." )

set( SimpleThreadPrint_SRCS "${ThreadExamples_SOURCE_DIR}/../SimpleThreadPrint/SimpleThreadPrint.cpp" )
set( SimpleThreadPrintLock_SRCS "${ThreadExamples_SOURCE_DIR}/../SimpleThreadPrintLock/SimpleThreadPrintLock.cpp" )
set( PeriodicThreadCallbacks_SRCS "${ThreadExamples_SOURCE_DIR}/../PeriodicThreadCallbacks/PeriodicThreadCallbacks.cpp" )

set( required_libs )

set( tmp_h3d_include_dirs )
findIncludeDirsAndLibrariesForH3DProjects ( PROJECT_NAMES H3DUtil
                                            INCLUDE_DIRS_OUTPUT_VAR tmp_h3d_include_dirs
                                            LIBRARIES_OUTPUT_VAR required_libs
                                            REQUIRED_PROJECTS H3DUtil )
include_directories( ${tmp_h3d_include_dirs} )

add_executable( SimpleThreadPrint ${SimpleThreadPrint_SRCS} )
add_executable( SimpleThreadPrintLock ${SimpleThreadPrintLock_SRCS} )
add_executable( PeriodicThreadCallbacks ${PeriodicThreadCallbacks_SRCS} )

# make the name of debug libraries end in _d.
set_target_properties( SimpleThreadPrint PROPERTIES DEBUG_POSTFIX "_d" )
set_target_properties( SimpleThreadPrintLock PROPERTIES DEBUG_POSTFIX "_d" )
set_target_properties( PeriodicThreadCallbacks PROPERTIES DEBUG_POSTFIX "_d" )

target_link_libraries( SimpleThreadPrint ${required_libs} )
target_link_libraries( SimpleThreadPrintLock ${required_libs} )
target_link_libraries( PeriodicThreadCallbacks ${required_libs} )

getDefaultH3DOutputDirectoryName( default_bin_install default_lib_install )

install( TARGETS SimpleThreadPrint
         LIBRARY DESTINATION ${default_lib_install} COMPONENT HAPI_cpack_examples_runtime
         RUNTIME DESTINATION ${default_bin_install} COMPONENT HAPI_cpack_examples_runtime )

install( TARGETS SimpleThreadPrintLock
         LIBRARY DESTINATION ${default_lib_install} COMPONENT HAPI_cpack_examples_runtime
         RUNTIME DESTINATION ${default_bin_install} COMPONENT HAPI_cpack_examples_runtime )

install( TARGETS PeriodicThreadCallbacks
         LIBRARY DESTINATION ${default_lib_install} COMPONENT HAPI_cpack_examples_runtime
         RUNTIME DESTINATION ${default_bin_install} COMPONENT HAPI_cpack_examples_runtime )
