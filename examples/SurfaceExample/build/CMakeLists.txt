cmake_minimum_required( VERSION 2.8.7 )
cmake_policy( VERSION 2.8.7 )

# The name of our project is "SurfaceExample".  CMakeLists files in this project can
# refer to the root source directory of the project as ${SurfaceExample_SOURCE_DIR} and
# to the root binary directory of the project as ${SurfaceExample_BINARY_DIR}.
project( SurfaceExample )

set( CMAKE_MODULE_PATH "${SurfaceExample_SOURCE_DIR}/../../../build/modules" )

include( H3DCommonFunctions )
handleCommonCacheVar( CMAKE_INSTALL_PREFIX ${SurfaceExample_SOURCE_DIR}/../.. )
setupRPathForLib()

# Create a library called "SurfaceExample" which includes the source file "SurfaceExample.cpp".
# The extension is already found.  Any number of sources could be listed here.

set( SurfaceExample_SRCS "${SurfaceExample_SOURCE_DIR}/../SurfaceDemo.cpp" )

include_directories( ${SurfaceExample_SOURCE_DIR}/.. )

set( required_libs )

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

  if( H3DUtil_FOUND )
    include_directories( ${H3DUtil_INCLUDE_DIRS} ) 
    set( required_libs ${required_libs} ${H3DUtil_LIBRARIES} )
  endif()
endif()

if( TARGET HAPI )
  include_directories( ${HAPI_INCLUDE_DIR} ) 
  set( required_libs ${required_libs} HAPI )
else()
  #HAPI
  find_package( HAPI REQUIRED )

  if( HAPI_FOUND )
    include_directories( ${HAPI_INCLUDE_DIRS} ) 
    set( required_libs ${required_libs} ${HAPI_LIBRARIES} )
  endif()
endif()

add_executable( SurfaceExample ${SurfaceExample_SRCS} )

# make the name of debug libraries end in _d.
set_target_properties( SurfaceExample PROPERTIES DEBUG_POSTFIX "_d" )

target_link_libraries( SurfaceExample ${required_libs} ${optional_libs} )

getDefaultH3DOutputDirectoryName( default_bin_install default_lib_install )

install( TARGETS SurfaceExample
         LIBRARY DESTINATION ${default_lib_install} COMPONENT HAPI_cpack_examples_runtime
         RUNTIME DESTINATION ${default_bin_install} COMPONENT HAPI_cpack_examples_runtime )
