if( WIN32 )
  cmake_minimum_required( VERSION 2.6.0 )
endif()
if( COMMAND cmake_policy )
  if( POLICY CMP0003 )
    cmake_policy( SET CMP0003 NEW )
  endif()
endif()
# The name of our project is "HAPIDemo".  CMakeLists files in this project can
# refer to the root source directory of the project as ${HAPIDemo_SOURCE_DIR} and
# to the root binary directory of the project as ${HAPIDemo_BINARY_DIR}.
project( HAPIDemo )

# set the install directory on Windows
if( WIN32 AND CMAKE_INSTALL_PREFIX_INITIALIZED_TO_DEFAULT AND NOT H3D_CMAKE_INSTALL_PREFIX_ALREADY_SET )
  set( CMAKE_INSTALL_PREFIX ${HAPIDemo_SOURCE_DIR}/../.. CACHE PATH "Install path prefix, prepended onto install directories." FORCE )
  set( H3D_CMAKE_INSTALL_PREFIX_ALREADY_SET TRUE )
endif()

# use, i.e. don't skip the full RPATH for the build tree
set( CMAKE_SKIP_BUILD_RPATH FALSE )

# when building, don't use the install RPATH already
# (but later on when installing)
set( CMAKE_BUILD_WITH_INSTALL_RPATH FALSE ) 

# add the automatically determined parts of the RPATH
# which point to directories outside the build tree to the install RPATH
set( CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE )

# the RPATH to be used when installing, but only if it's not a system directory
list( FIND CMAKE_PLATFORM_IMPLICIT_LINK_DIRECTORIES "${CMAKE_INSTALL_PREFIX}/lib" isSystemDir )
if( "${isSystemDir}" STREQUAL "-1" )
   set( CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/lib" )
endif()

# Create a library called "Hello" which include sourcefiles.
# The extension is already found.  Any number of sources could be listed here.

set( HAPIDemo_SRCS "${HAPIDemo_SOURCE_DIR}/../ForceFieldWidgetsPage.cpp"
                   "${HAPIDemo_SOURCE_DIR}/../ForceFieldWidgetsPage.h"
                   "${HAPIDemo_SOURCE_DIR}/../HAPIDemo.cpp"
                   "${HAPIDemo_SOURCE_DIR}/../HAPIDemo.h"
                   "${HAPIDemo_SOURCE_DIR}/../HapticShapeConstraintWidgetsPage.cpp"
                   "${HAPIDemo_SOURCE_DIR}/../HapticShapeConstraintWidgetsPage.h"
                   "${HAPIDemo_SOURCE_DIR}/../PositionFunctionWidgetsPage.cpp"
                   "${HAPIDemo_SOURCE_DIR}/../PositionFunctionWidgetsPage.h"
                   "${HAPIDemo_SOURCE_DIR}/../SpringWidgetsPage.cpp"
                   "${HAPIDemo_SOURCE_DIR}/../SpringWidgetsPage.h"
                   "${HAPIDemo_SOURCE_DIR}/../TimeFunctionWidgetsPage.cpp"
                   "${HAPIDemo_SOURCE_DIR}/../TimeFunctionWidgetsPage.h"
                   "${HAPIDemo_SOURCE_DIR}/../ViscosityWidgetsPage.cpp"
                   "${HAPIDemo_SOURCE_DIR}/../ViscosityWidgetsPage.h" )

include_directories( ${HAPIDemo_SOURCE_DIR}/.. )

set( required_libs )

set( CMAKE_MODULE_PATH "${HAPIDemo_SOURCE_DIR}/../../../build/modules" )

if( H3D_USE_DEPENDENCIES_ONLY )
  # The variables set here must be set by the CMakeLists.txt that sets H3D_USE_DEPENDENCIES_ONLY to true.
  include_directories( ${EXTERNAL_INCLUDE_DIR} ) 
endif()

if( TARGET H3DUtil )
  include_directories( ${H3DUTIL_INCLUDE_DIR} ) 
  set( required_libs ${required_libs} H3DUtil )
else()
  #H3DUtil
  find_package( H3DUtil REQUIRED )

  if( H3DUTIL_FOUND )
    include_directories( ${H3DUTIL_INCLUDE_DIR} ) 
    set( required_libs ${required_libs} ${H3DUTIL_LIBRARIES} )
  endif()
endif()

if( TARGET HAPI )
  include_directories( ${HAPI_INCLUDE_DIR} ) 
  set( required_libs ${required_libs} HAPI )
else()
  #HAPI
  find_package( HAPI REQUIRED )

  if( HAPI_FOUND )
    include_directories( ${HAPI_INCLUDE_DIR} ) 
    set( required_libs ${required_libs} ${HAPI_LIBRARIES} )
  endif()
endif()

set( WXWINDOWS_USE_GL 1 )
find_package( wxWidgets REQUIRED COMPONENTS base core )
if( wxWidgets_FOUND )
  if( WIN32 )
    include_directories( ${wxWidgets_INCLUDE_DIR} ) 
  else()
    include( ${wxWidgets_USE_FILE} )
  endif()
  set( required_libs ${required_libs} ${wxWidgets_LIBRARIES} )
endif()

add_executable( HAPIDemo WIN32 MACOSX_BUNDLE ${HAPIDemo_SRCS} )

# make the name of debug libraries end in _d.
set_target_properties( HAPIDemo PROPERTIES DEBUG_POSTFIX "_d" )

if( MSVC )
  
  set( HAPIDemo_COMPILE_FLAGS "-DWXUSINGDLL" )
  
  # Treat wchar_t as built in type for all visual studio versions.
  # This is default for every version above 7 (so far) but we still set it for all.
  set( HAPIDemo_COMPILE_FLAGS "${HAPIDemo_COMPILE_FLAGS} /Zc:wchar_t" )
  
  if( ${MSVC_VERSION} GREATER 1399 )
    # Remove compiler warnings about deprecation for visual studio versions 8 and above.
    set( HAPIDemo_COMPILE_FLAGS "${HAPIDemo_COMPILE_FLAGS} -D_CRT_SECURE_NO_DEPRECATE" )
  endif()
  
  if( ${MSVC_VERSION} GREATER 1499 )
    # Build using several threads for visual studio versions 9 and above.
    set( HAPIDemo_COMPILE_FLAGS "${HAPIDemo_COMPILE_FLAGS} /MP" )
  endif()
  
  if( ${MSVC_VERSION} LESS 1600 )
    # Turn off optimization for visual studio versions 8 and below.
    # Old setting which I (Markus) have no idea why it was added.
    set( HAPIDemo_COMPILE_FLAGS "${HAPIDemo_COMPILE_FLAGS} /Od" )
  endif()

  set_target_properties( HAPIDemo PROPERTIES COMPILE_FLAGS "${HAPIDemo_COMPILE_FLAGS}" )
  set_target_properties( HAPIDemo PROPERTIES LINK_FLAGS_DEBUG "/NODEFAULTLIB:msvcrt" )
endif()

target_link_libraries( HAPIDemo ${required_libs} ${optional_libs} )

set( default_bin_install "bin" )
set( default_lib_install "lib" )

if( WIN32 )
  set( default_bin_install "bin32" )
  set( default_lib_install "lib32" )
  if( CMAKE_SIZEOF_VOID_P EQUAL 8 )
    set( default_bin_install "bin64" )
    set( default_lib_install "lib64" )
  endif()
endif()

install( TARGETS HAPIDemo
         LIBRARY DESTINATION ${default_lib_install} COMPONENT HAPI_cpack_examples_runtime
         RUNTIME DESTINATION ${default_bin_install} COMPONENT HAPI_cpack_examples_runtime )
