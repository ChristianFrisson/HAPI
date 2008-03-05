# The name of our project is "HELLO".  CMakeLists files in this project can
# refer to the root source directory of the project as ${HELLO_SOURCE_DIR} and
# to the root binary directory of the project as ${HELLO_BINARY_DIR}.
PROJECT (HAPI)

# Create a library called "Hello" which includes the source file "hello.cxx".
# The extension is already found.  Any number of sources could be listed here.

AUX_SOURCE_DIRECTORY(../src HAPI_SRCS)
AUX_SOURCE_DIRECTORY(../OpenHapticsRenderer/src HAPI_SRCS)
AUX_SOURCE_DIRECTORY(../Chai3DRenderer/src HAPI_SRCS)

SET(optionalLibs)

INCLUDE_DIRECTORIES( ../include ../OpenHapticsRenderer/include
../Chai3DRenderer/include ${HAPI_SOURCE_DIR}/../../H3DUtil/include )


SET(CMAKE_SYSTEM_INCLUDE_PATH ${CMAKE_SYSTEM_INCLUDE_PATH}
"$ENV{H3D_EXTERNAL_ROOT}\\include" 
"$ENV{H3D_EXTERNAL_ROOT}\\include\\pthread" 
../../External/include 
../../External/include/pthread 
)

SET(CMAKE_SYSTEM_LIBRARY_PATH ${CMAKE_SYSTEM_LIBRARY_PATH}
"$ENV{H3D_EXTERNAL_ROOT}\\lib" ../../External/lib )

SET(requiredLibs H3DUtil )

IF(WIN32)
  SET(requiredLibs ${requiredLibs} winmm.lib )
ENDIF(WIN32)

SET(CMAKE_MODULE_PATH ${HAPI_SOURCE_DIR}/modules )

FIND_PACKAGE(PTHREAD REQUIRED)

IF(PTHREAD_FOUND)
  INCLUDE_DIRECTORIES( ${PTHREAD_INCLUDE_DIR} ) 
  SET(requiredLibs ${requiredLibs} ${PTHREAD_LIBRARIES} )
ENDIF(PTHREAD_FOUND)

#FIND_PACKAGE(H3DUtil REQUIRED)

 

#IF(H3DUTIL_FOUND)
#  INCLUDE_DIRECTORIES( ${H3DUTIL_INCLUDE_DIR} ) 
#  SET(requiredLibs ${requiredLibs} ${H3DUTIL_LIBRARIES} )
#ENDIF(H3DUTIL_FOUND)

FIND_PACKAGE(OpenHaptics)

IF(OPENHAPTICS_FOUND)
  SET(HAVE_OPENHAPTICS 1)
  INCLUDE_DIRECTORIES( ${OPENHAPTICS_INCLUDE_DIR} ) 
  SET(optionalLibs ${optionalLibs} ${OPENHAPTICS_LIBRARIES} )
ENDIF(OPENHAPTICS_FOUND)

FIND_PACKAGE(Chai3D)

IF(CHAI3D_FOUND)
  SET(HAVE_CHAI3D 1)
  INCLUDE_DIRECTORIES( ${CHAI3D_INCLUDE_DIR} ) 
  SET(optionalLibs ${optionalLibs} ${CHAI3D_LIBRARIES} )
ENDIF(CHAI3D_FOUND)

FIND_PACKAGE(DHD)

IF(DHD_FOUND)
  SET(HAVE_DHD 1)
  INCLUDE_DIRECTORIES( ${DHD_INCLUDE_DIR} ) 
  SET(optionalLibs ${optionalLibs} ${DHD_LIBRARIES} )
ENDIF(DHD_FOUND)

FIND_PACKAGE(Haptik)

IF(HAPTIK_FOUND)
  SET(HAVE_HAPTIK_LIBRARY 1)
  INCLUDE_DIRECTORIES( ${HAPTIK_INCLUDE_DIR} ) 
  SET(optionalLibs ${optionalLibs} ${HAPTIK_LIBRARIES} )
ENDIF(HAPTIK_FOUND)


FIND_PACKAGE(OpenGL REQUIRED)

IF(OPENGL_FOUND)
  SET(HAVE_OPENGL 1)
  INCLUDE_DIRECTORIES( ${OPENGL_INCLUDE_DIR} ) 
  SET(requiredLibs ${requiredLibs} ${OPENGL_LIBRARIES} )
ENDIF(OPENGL_FOUND)

ADD_LIBRARY(HAPI SHARED ${HAPI_SRCS})
SET_TARGET_PROPERTIES( HAPI PROPERTIES COMPILE_FLAGS "-DHAPI_EXPORTS" )
TARGET_LINK_LIBRARIES( HAPI ${requiredLibs} ${optionalLibs} )

CONFIGURE_FILE( ../include/HAPI/HAPI.cmake ../include/HAPI/HAPI.h )

INSTALL( TARGETS HAPI 
	 LIBRARY DESTINATION lib
	 RUNTIME DESTINATION bin )

INSTALL( DIRECTORY ../include/HAPI ../OpenHapticsRenderer/include/HAPI ../Chai3DRenderer/include/HAPI
	 DESTINATION /usr/local/include 
	 PATTERN .svn EXCLUDE )
    
ADD_SUBDIRECTORY( ${HAPI_SOURCE_DIR}/../../H3DUtil/build
${HAPI_SOURCE_DIR}/../../H3DUtil/build )
ADD_DEPENDENCIES( HAPI H3DUtil )
