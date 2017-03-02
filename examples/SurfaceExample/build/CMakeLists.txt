if(WIN32)
  cmake_minimum_required(VERSION 2.6.0)
endif(WIN32)
# The name of our project is "SurfaceExample".  CMakeLists files in this project can
# refer to the root source directory of the project as ${SurfaceExample_SOURCE_DIR} and
# to the root binary directory of the project as ${SurfaceExample_BINARY_DIR}.
project (SurfaceExample)

# set the install directory on Windows
if( WIN32 AND CMAKE_INSTALL_PREFIX_INITIALIZED_TO_DEFAULT AND NOT H3D_CMAKE_INSTALL_PREFIX_ALREADY_SET )
  set( CMAKE_INSTALL_PREFIX ${SurfaceExample_SOURCE_DIR}/../.. CACHE PATH "Install path prefix, prepended onto install directories." FORCE )
  set( H3D_CMAKE_INSTALL_PREFIX_ALREADY_SET TRUE )
endif( WIN32 AND CMAKE_INSTALL_PREFIX_INITIALIZED_TO_DEFAULT AND NOT H3D_CMAKE_INSTALL_PREFIX_ALREADY_SET )

# use, i.e. don't skip the full RPATH for the build tree
set(CMAKE_SKIP_BUILD_RPATH FALSE)

# when building, don't use the install RPATH already
# (but later on when installing)
set(CMAKE_BUILD_WITH_INSTALL_RPATH FALSE) 

# add the automatically determined parts of the RPATH
# which point to directories outside the build tree to the install RPATH
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)

# the RPATH to be used when installing, but only if it's not a system directory
list(FIND CMAKE_PLATFORM_IMPLICIT_LINK_DIRECTORIES "${CMAKE_INSTALL_PREFIX}/lib" isSystemDir)
if("${isSystemDir}" STREQUAL "-1")
   set(CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/lib")
endif("${isSystemDir}" STREQUAL "-1")

# Create a library called "SurfaceExample" which includes the source file "SurfaceExample.cpp".
# The extension is already found.  Any number of sources could be listed here.

set( SurfaceExample_SRCS "${SurfaceExample_SOURCE_DIR}/../SurfaceDemo.cpp" )

include_directories( ${SurfaceExample_SOURCE_DIR}/.. )

set(requiredLibs)

set(CMAKE_MODULE_PATH ${SurfaceExample_SOURCE_DIR}/../../../build/modules/sharedModules )

if( H3D_USE_DEPENDENCIES_ONLY )
  # The variables set here must be set by the CMakeLists.txt that sets H3D_USE_DEPENDENCIES_ONLY to true.
  include_directories( ${EXTERNAL_INCLUDE_DIR} ) 
endif( H3D_USE_DEPENDENCIES_ONLY )

if( TARGET H3DUtil )
  include_directories( ${H3DUTIL_INCLUDE_DIR} ) 
  set( requiredLibs ${requiredLibs} H3DUtil )
else( TARGET H3DUtil )
  #H3DUtil
  find_package(H3DUtil REQUIRED)

  if(H3DUTIL_FOUND)
    include_directories( ${H3DUTIL_INCLUDE_DIR} ) 
    set(requiredLibs ${requiredLibs} ${H3DUTIL_LIBRARIES} )
  endif(H3DUTIL_FOUND)
endif( TARGET H3DUtil )

if( TARGET HAPI )
  include_directories( ${HAPI_INCLUDE_DIR} ) 
  set( requiredLibs ${requiredLibs} HAPI )
else( TARGET HAPI )
  #HAPI
  find_package(HAPI REQUIRED)

  if(HAPI_FOUND)
    include_directories( ${HAPI_INCLUDE_DIR} ) 
    set(requiredLibs ${requiredLibs} ${HAPI_LIBRARIES} )
  endif(HAPI_FOUND)
endif( TARGET HAPI )

add_executable(SurfaceExample ${SurfaceExample_SRCS})

# make the name of debug libraries end in _d.
set_target_properties( SurfaceExample PROPERTIES DEBUG_POSTFIX "_d" )

target_link_libraries( SurfaceExample ${requiredLibs} ${optionalLibs} )

set( DEFAULT_BIN_INSTALL "bin" )
set( DEFAULT_LIB_INSTALL "lib" )

if( WIN32 )
  set( DEFAULT_BIN_INSTALL "bin32" )
  set( DEFAULT_LIB_INSTALL "lib32" )
  if( CMAKE_SIZEOF_VOID_P EQUAL 8 )
    set( DEFAULT_BIN_INSTALL "bin64" )
    set( DEFAULT_LIB_INSTALL "lib64" )
  endif( CMAKE_SIZEOF_VOID_P EQUAL 8 )
endif( WIN32 )

install( TARGETS SurfaceExample
	 LIBRARY DESTINATION ${DEFAULT_LIB_INSTALL} COMPONENT HAPI_cpack_examples_runtime
	 RUNTIME DESTINATION ${DEFAULT_BIN_INSTALL} COMPONENT HAPI_cpack_examples_runtime )
