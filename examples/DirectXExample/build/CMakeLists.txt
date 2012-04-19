IF(WIN32)
  cmake_minimum_required(VERSION 2.6.0)
ENDIF(WIN32)
if( COMMAND cmake_policy )
  if( POLICY CMP0003 )
    cmake_policy( SET CMP0003 NEW )
  endif( POLICY CMP0003 )
endif( COMMAND cmake_policy )
# The name of our project is "DirectXExample".  CMakeLists files in this project can
# refer to the root source directory of the project as ${DirectXExample_SOURCE_DIR} and
# to the root binary directory of the project as ${DirectXExample_BINARY_DIR}.
project (DirectXExample)

# Create a library called "DirectXExample" which includes the source files.
# The extension is already found.  Any number of sources could be listed here.

SET( DirectXExample_SRCS "${DirectXExample_SOURCE_DIR}/../src/main.cpp" )

INCLUDE_DIRECTORIES( ${DirectXExample_SOURCE_DIR}/../src )

SET(requiredLibs)

SET(CMAKE_MODULE_PATH ${DirectXExample_SOURCE_DIR}/../../../build/modules )

IF( H3D_USE_DEPENDECIES_ONLY )
  # The variables set here must be set by the CMakeLists.txt that sets H3D_USE_DEPENDECIES_ONLY to true.
  IF(PTHREAD_FOUND)
    INCLUDE_DIRECTORIES( ${PTHREAD_INCLUDE_DIR} ) 
  ENDIF(PTHREAD_FOUND)
  INCLUDE_DIRECTORIES( ${EXTERNAL_INCLUDE_DIR} ) 
  INCLUDE_DIRECTORIES( ${H3DUTIL_INCLUDE_DIR} ) 
  INCLUDE_DIRECTORIES( ${HAPI_INCLUDE_DIR} ) 
  
  SET( requiredLibs ${requiredLibs} H3DUtil HAPI )
ELSE( H3D_USE_DEPENDECIES_ONLY )
  #H3DUtil
  FIND_PACKAGE(H3DUtil REQUIRED)

  IF(H3DUTIL_FOUND)
    INCLUDE_DIRECTORIES( ${H3DUTIL_INCLUDE_DIR} ) 
    SET(requiredLibs ${requiredLibs} ${H3DUTIL_LIBRARIES} )
  ENDIF(H3DUTIL_FOUND)

  #HAPI
  FIND_PACKAGE(HAPI REQUIRED)

  IF(HAPI_FOUND)
    INCLUDE_DIRECTORIES( ${HAPI_INCLUDE_DIR} ) 
    SET(requiredLibs ${requiredLibs} ${HAPI_LIBRARIES} )
  ENDIF(HAPI_FOUND)
ENDIF( H3D_USE_DEPENDECIES_ONLY )

FIND_PACKAGE( DirectX REQUIRED )
IF( DirectX_FOUND )
  INCLUDE_DIRECTORIES( ${DirectX_INCLUDE_DIR} )
  SET( requiredLibs ${requiredLibs} ${DirectX_LIBRARIES} )
ENDIF( DirectX_FOUND )

ADD_EXECUTABLE(DirectXExample WIN32 ${DirectXExample_SRCS})

# make the name of debug libraries end in _d.
SET_TARGET_PROPERTIES( DirectXExample PROPERTIES DEBUG_POSTFIX "_d" )

TARGET_LINK_LIBRARIES( DirectXExample ${requiredLibs} )

# set the install directory to the H3D directory on Windows
IF(WIN32) 
  IF( CMAKE_INSTALL_PREFIX_INITIALIZED_TO_DEFAULT AND NOT H3D_USE_DEPENDENCIES_ONLY)
    SET( CMAKE_INSTALL_PREFIX ${DirectXExample_SOURCE_DIR}/../.. CACHE PATH "Install path prefix, prepended onto install directories." FORCE )
  ENDIF( CMAKE_INSTALL_PREFIX_INITIALIZED_TO_DEFAULT AND NOT H3D_USE_DEPENDENCIES_ONLY)
ENDIF(WIN32)

INSTALL( TARGETS DirectXExample
	 LIBRARY DESTINATION lib
	 RUNTIME DESTINATION bin )
