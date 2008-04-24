cmake_minimum_required(VERSION 2.4.8)
# The name of our project is "DirectXExample".  CMakeLists files in this project can
# refer to the root source directory of the project as ${DirectXExample_SOURCE_DIR} and
# to the root binary directory of the project as ${DirectXExample_BINARY_DIR}.
project (DirectXExample)

# Create a library called "DirectXExample" which includes the source files.
# The extension is already found.  Any number of sources could be listed here.

AUX_SOURCE_DIRECTORY(../src DirectXExample_SRCS)

INCLUDE_DIRECTORIES( ../src )

SET(requiredLibs)

SET(CMAKE_MODULE_PATH ${DirectXExample_SOURCE_DIR}/../../../build/modules )

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

FIND_PACKAGE( DirectX REQUIRED )
IF( DirectX_FOUND )
  INCLUDE_DIRECTORIES( ${DirectX_INCLUDE_DIR} )
  SET( requiredLibs ${requiredLibs} ${DirectX_LIBRARIES} )
ENDIF( DirectX_FOUND )

ADD_EXECUTABLE(DirectXExample WIN32 ${DirectXExample_SRCS})

TARGET_LINK_LIBRARIES( DirectXExample ${requiredLibs} )

# set the install directory to the H3D directory on Windows
IF(WIN32) 
 SET( CMAKE_INSTALL_PREFIX ${DirectXExample_SOURCE_DIR}/../..)
ENDIF(WIN32)

INSTALL( TARGETS DirectXExample
	 LIBRARY DESTINATION lib
	 RUNTIME DESTINATION bin )
