cmake_minimum_required(VERSION 2.4.8)
# The name of our project is "SurfaceExample".  CMakeLists files in this project can
# refer to the root source directory of the project as ${SurfaceExample_SOURCE_DIR} and
# to the root binary directory of the project as ${SurfaceExample_BINARY_DIR}.
project (SurfaceExample)

# Create a library called "SurfaceExample" which includes the source file "SurfaceExample.cpp".
# The extension is already found.  Any number of sources could be listed here.

AUX_SOURCE_DIRECTORY(${SurfaceExample_SOURCE_DIR}/.. SurfaceExample_SRCS)

INCLUDE_DIRECTORIES( ${SurfaceExample_SOURCE_DIR}/.. )

SET(requiredLibs)

SET(CMAKE_MODULE_PATH ${SurfaceExample_SOURCE_DIR}/../../../build/modules )

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

ADD_EXECUTABLE(SurfaceExample ${SurfaceExample_SRCS})

# make the name of debug libraries end in _d.
SET_TARGET_PROPERTIES( SurfaceExample PROPERTIES DEBUG_POSTFIX "_d" )

TARGET_LINK_LIBRARIES( SurfaceExample ${requiredLibs} ${optionalLibs} )

# set the install directory to the H3D directory on Windows
IF(WIN32) 
 SET( CMAKE_INSTALL_PREFIX ${SurfaceExample_SOURCE_DIR}/../..)
ENDIF(WIN32)

INSTALL( TARGETS SurfaceExample
	 LIBRARY DESTINATION lib
	 RUNTIME DESTINATION bin )

