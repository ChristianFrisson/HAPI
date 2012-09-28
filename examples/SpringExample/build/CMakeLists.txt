IF(WIN32)
  cmake_minimum_required(VERSION 2.6.0)
ENDIF(WIN32)
# The name of our project is "SpringExample".  CMakeLists files in this project can
# refer to the root source directory of the project as ${SpringExample_SOURCE_DIR} and
# to the root binary directory of the project as ${SpringExample_BINARY_DIR}.
PROJECT(SpringExample)

# set the install directory on Windows
IF( WIN32 AND CMAKE_INSTALL_PREFIX_INITIALIZED_TO_DEFAULT AND NOT H3D_CMAKE_INSTALL_PREFIX_ALREADY_SET )
  SET( CMAKE_INSTALL_PREFIX ${SpringExample_SOURCE_DIR}/../.. CACHE PATH "Install path prefix, prepended onto install directories." FORCE )
  SET( H3D_CMAKE_INSTALL_PREFIX_ALREADY_SET TRUE )
ENDIF( WIN32 AND CMAKE_INSTALL_PREFIX_INITIALIZED_TO_DEFAULT AND NOT H3D_CMAKE_INSTALL_PREFIX_ALREADY_SET )

# Create a library called "SpringExample" which includes the source file "SpringExample.cpp".
# The extension is already found.  Any number of sources could be listed here.

SET( SpringExample_SRCS "${SpringExample_SOURCE_DIR}/../SpringExample.cpp" )

INCLUDE_DIRECTORIES( ${SpringExample_SOURCE_DIR}/.. )

SET(requiredLibs)

SET(CMAKE_MODULE_PATH ${SpringExample_SOURCE_DIR}/../../../build/modules )

IF( TARGET H3DUtil )
  INCLUDE_DIRECTORIES( ${H3DUTIL_INCLUDE_DIR} ) 
  SET( requiredLibs ${requiredLibs} H3DUtil )
ELSE( TARGET H3DUtil )
  #H3DUtil
  FIND_PACKAGE(H3DUtil REQUIRED)

  IF(H3DUTIL_FOUND)
    INCLUDE_DIRECTORIES( ${H3DUTIL_INCLUDE_DIR} ) 
    SET(requiredLibs ${requiredLibs} ${H3DUTIL_LIBRARIES} )
  ENDIF(H3DUTIL_FOUND)
ENDIF( TARGET H3DUtil )

IF( TARGET HAPI )
  INCLUDE_DIRECTORIES( ${HAPI_INCLUDE_DIR} ) 
  SET( requiredLibs ${requiredLibs} HAPI )
ELSE( TARGET HAPI )
  #HAPI
  FIND_PACKAGE(HAPI REQUIRED)

  IF(HAPI_FOUND)
    INCLUDE_DIRECTORIES( ${HAPI_INCLUDE_DIR} ) 
    SET(requiredLibs ${requiredLibs} ${HAPI_LIBRARIES} )
  ENDIF(HAPI_FOUND)
ENDIF( TARGET HAPI )

IF( H3D_USE_DEPENDENCIES_ONLY )
  # The variables set here must be set by the CMakeLists.txt that sets H3D_USE_DEPENDENCIES_ONLY to true.
  IF(PTHREAD_FOUND)
    INCLUDE_DIRECTORIES( ${PTHREAD_INCLUDE_DIR} ) 
  ENDIF(PTHREAD_FOUND)
  INCLUDE_DIRECTORIES( ${EXTERNAL_INCLUDE_DIR} ) 
ENDIF( H3D_USE_DEPENDENCIES_ONLY )

ADD_EXECUTABLE(SpringExample ${SpringExample_SRCS})

# make the name of debug libraries end in _d.
SET_TARGET_PROPERTIES( SpringExample PROPERTIES DEBUG_POSTFIX "_d" )

TARGET_LINK_LIBRARIES( SpringExample ${requiredLibs} ${optionalLibs} )

INSTALL( TARGETS SpringExample
	 LIBRARY DESTINATION lib COMPONENT HAPI_cpack_examples_runtime
	 RUNTIME DESTINATION bin COMPONENT HAPI_cpack_examples_runtime )
