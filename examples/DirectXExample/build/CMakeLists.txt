if( WIN32 )
  cmake_minimum_required( VERSION 2.6.0 )
endif()
if( COMMAND cmake_policy )
  if( POLICY CMP0003 )
    cmake_policy( SET CMP0003 NEW )
  endif()
endif()
# The name of our project is "DirectXExample".  CMakeLists files in this project can
# refer to the root source directory of the project as ${DirectXExample_SOURCE_DIR} and
# to the root binary directory of the project as ${DirectXExample_BINARY_DIR}.
project( DirectXExample )

# set the install directory on Windows
if( WIN32 AND CMAKE_INSTALL_PREFIX_INITIALIZED_TO_DEFAULT AND NOT H3D_CMAKE_INSTALL_PREFIX_ALREADY_SET )
  set( CMAKE_INSTALL_PREFIX ${DirectXExample_SOURCE_DIR}/../.. CACHE PATH "Install path prefix, prepended onto install directories." FORCE )
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

# Create a library called "DirectXExample" which includes the source files.
# The extension is already found.  Any number of sources could be listed here.

set( DirectXExample_SRCS "${DirectXExample_SOURCE_DIR}/../src/main.cpp" )

include_directories( ${DirectXExample_SOURCE_DIR}/../src )

set( required_libs )

set( CMAKE_MODULE_PATH "${DirectXExample_SOURCE_DIR}/../../../build/modules" )

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

find_package( DirectX REQUIRED )
if( DirectX_FOUND )
  include_directories( ${DirectX_INCLUDE_DIRS} )
  set( required_libs ${required_libs} ${DirectX_LIBRARIES} )
endif()

add_executable( DirectXExample WIN32 ${DirectXExample_SRCS} )

# make the name of debug libraries end in _d.
set_target_properties( DirectXExample PROPERTIES DEBUG_POSTFIX "_d" )

target_link_libraries( DirectXExample ${required_libs} )

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

install( TARGETS DirectXExample
         LIBRARY DESTINATION ${default_lib_install} COMPONENT HAPI_cpack_examples_runtime
         RUNTIME DESTINATION ${default_bin_install} COMPONENT HAPI_cpack_examples_runtime )
