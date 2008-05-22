IF(WIN32)
  cmake_minimum_required(VERSION 2.6.0)
ENDIF(WIN32)
# The name of our project is "SpringExample".  CMakeLists files in this project can
# refer to the root source directory of the project as ${SpringExample_SOURCE_DIR} and
# to the root binary directory of the project as ${SpringExample_BINARY_DIR}.
PROJECT(SpringExample)

# Create a library called "SpringExample" which includes the source file "SpringExample.cpp".
# The extension is already found.  Any number of sources could be listed here.

AUX_SOURCE_DIRECTORY(.. SpringExample_SRCS)

INCLUDE_DIRECTORIES( ${SpringExample_SOURCE_DIR}/.. )

SET(requiredLibs)

SET(CMAKE_MODULE_PATH ${SpringExample_SOURCE_DIR}/../../../build/modules )

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

ADD_EXECUTABLE(SpringExample ${SpringExample_SRCS})

# make the name of debug libraries end in _d.
SET_TARGET_PROPERTIES( SpringExample PROPERTIES DEBUG_POSTFIX "_d" )

TARGET_LINK_LIBRARIES( SpringExample ${requiredLibs} ${optionalLibs} )

# set the install directory to the H3D directory on Windows
IF(WIN32) 
 SET( CMAKE_INSTALL_PREFIX ${SpringExample_SOURCE_DIR}/../..)
ENDIF(WIN32)

INSTALL( TARGETS SpringExample
	 LIBRARY DESTINATION lib
	 RUNTIME DESTINATION bin )

