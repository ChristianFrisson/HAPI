cmake_minimum_required(VERSION 2.4.8)
# The name of our project is "FeedbackBufferCollector".  CMakeLists files in this project can
# refer to the root source directory of the project as ${FeedbackBufferCollector_SOURCE_DIR} and
# to the root binary directory of the project as ${FeedbackBufferCollector_BINARY_DIR}.
project (FeedbackBufferCollector)

# Create a library called "FeedbackBufferCollector" which includes the source files.
# The extension is already found.  Any number of sources could be listed here.

AUX_SOURCE_DIRECTORY(.. FeedbackBufferCollector_SRCS)

INCLUDE_DIRECTORIES( .. )

SET(requiredLibs)

SET(CMAKE_MODULE_PATH ${FeedbackBufferCollector_SOURCE_DIR}/../../../build/modules )

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

FIND_PACKAGE(PTHREAD REQUIRED)
IF(PTHREAD_FOUND)
  INCLUDE_DIRECTORIES( ${PTHREAD_INCLUDE_DIR} ) 
  SET(requiredLibs ${requiredLibs} ${PTHREAD_LIBRARIES} )
ENDIF(PTHREAD_FOUND)

IF(WIN32)
  FIND_PACKAGE(GLUTWin REQUIRED)
  IF(GLUT_FOUND)
    INCLUDE_DIRECTORIES( ${GLUT_INCLUDE_DIR} ) 
    SET(requiredLibs ${requiredLibs} ${GLUT_LIBRARIES} )
  ENDIF(GLUT_FOUND)
ENDIF(WIN32)

IF(GLUT_FOUND)
ELSE(GLUT_FOUND)
  FIND_PACKAGE(GLUT REQUIRED)
  IF(GLUT_FOUND)
    INCLUDE_DIRECTORIES( ${GLUT_INCLUDE_DIR} ) 
    SET(requiredLibs ${requiredLibs} ${GLUT_LIBRARIES} )
  ENDIF(GLUT_FOUND)
ENDIF(GLUT_FOUND)

ADD_EXECUTABLE(FeedbackBufferCollector ${FeedbackBufferCollector_SRCS})

TARGET_LINK_LIBRARIES( FeedbackBufferCollector ${requiredLibs} )

# set the install directory to the H3D directory on Windows
IF(WIN32) 
 SET( CMAKE_INSTALL_PREFIX ${FeedbackBufferCollector_SOURCE_DIR}/../..)
ENDIF(WIN32)

INSTALL( TARGETS FeedbackBufferCollector
	 LIBRARY DESTINATION lib
	 RUNTIME DESTINATION bin )
