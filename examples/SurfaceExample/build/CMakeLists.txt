# The name of our project is "HELLO".  CMakeLists files in this project can
# refer to the root source directory of the project as ${HELLO_SOURCE_DIR} and
# to the root binary directory of the project as ${HELLO_BINARY_DIR}.
project (SurfaceExample)

# Create a library called "Hello" which includes the source file "hello.cxx".
# The extension is already found.  Any number of sources could be listed here.

AUX_SOURCE_DIRECTORY(.. SurfaceExample_SRCS)

INCLUDE_DIRECTORIES( .. )

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

TARGET_LINK_LIBRARIES( SurfaceExample ${requiredLibs} ${optionalLibs} )



