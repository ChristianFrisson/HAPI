PROJECT (HAPI)

SET( HAPI_MAJOR_VERSION 1 )
SET( HAPI_MINOR_VERSION 0 )
SET( HAPI_BUILD_VERSION 0 )

SET( HAPI_FULL_VERSION
${HAPI_MAJOR_VERSION}.${HAPI_MINOR_VERSION}.${HAPI_BUILD_VERSION} ) 

#add all sources
AUX_SOURCE_DIRECTORY(../src HAPI_SRCS)

# add the HAPI.rc resource file if Visual Studio
IF(MSVC)
  SET( HAPI_SRCS ${HAPI_SRCS} HAPI.rc )
ENDIF(MSVC)


AUX_SOURCE_DIRECTORY(../OpenHapticsRenderer/src OH_SRCS)
AUX_SOURCE_DIRECTORY(../Chai3DRenderer/src CHAI_SRCS)
INCLUDE_DIRECTORIES(../OpenHapticsRenderer/include
                    ../Chai3DRenderer/include )

# OpenHapticsRenderer and Chai3DRenderer are compiled as separate dlls 
# on Windows 
IF(WIN32)
  ADD_LIBRARY(HAPI SHARED ${HAPI_SRCS})
  IF(MSVC)
    SET( OH_SRCS ${OH_SRCS} OpenHapticsRenderer.rc )
    SET( CHAI_SRCS ${CHAI_SRCS} Chai3DRenderer.rc )
  ENDIF(MSVC)
  ADD_LIBRARY(OpenHapticsRenderer SHARED ${OH_SRCS})
  ADD_LIBRARY(Chai3DRenderer SHARED ${CHAI_SRCS})
ELSE(WIN32)
  ADD_LIBRARY(HAPI SHARED ${HAPI_SRCS} ${OH_SRCS} ${CHAI_SRCS} )
ENDIF(WIN32)

# make sure that the HAPI.rc contains the correct svn-version
IF( MSVC )
  EXECUTE_PROCESS( COMMAND FindSVNVersion ${HAPI_SOURCE_DIR}/../src/.svn/entries 
                   OUTPUT_VARIABLE HAPI_SVN_VERSION )
  # autogenerate resource files depending on the version
  CONFIGURE_FILE( HAPI.rc.cmake HAPI.rc )
  CONFIGURE_FILE( OpenHapticsRenderer.rc.cmake OpenHapticsRenderer.rc )
  CONFIGURE_FILE( Chai3DRenderer.rc.cmake Chai3DRenderer.rc )
  
  ADD_CUSTOM_COMMAND( TARGET HAPI 
                      PRE_BUILD 
                      COMMAND UpdateResourceFile 
                      ARGS HAPI HAPI.rc HAPI.rc.cmake
                      ${HAPI_MAJOR_VERSION} ${HAPI_MINOR_VERSION}
                      ${HAPI_BUILD_VERSION}
                      ${HAPI_SOURCE_DIR}/../src/.svn/entries )

  ADD_CUSTOM_COMMAND( TARGET OpenHapticsRenderer
                      PRE_BUILD 
                      COMMAND UpdateResourceFile 
                      ARGS HAPI OpenHapticsRenderer.rc OpenHapticsRenderer.rc.cmake
                      ${HAPI_MAJOR_VERSION} ${HAPI_MINOR_VERSION}
                      ${HAPI_BUILD_VERSION}
                      ${HAPI_SOURCE_DIR}/../src/.svn/entries )

  ADD_CUSTOM_COMMAND( TARGET Chai3DRenderer
                      PRE_BUILD 
                      COMMAND UpdateResourceFile 
                      ARGS HAPI Chai3DRenderer.rc Chai3DRenderer.rc.cmake
                      ${HAPI_MAJOR_VERSION} ${HAPI_MINOR_VERSION}
                      ${HAPI_BUILD_VERSION}
                      ${HAPI_SOURCE_DIR}/../src/.svn/entries )
ENDIF(MSVC)

# set the install directory to the H3D directory on Windows
IF(WIN32) 
 SET( CMAKE_INSTALL_PREFIX ${HAPI_SOURCE_DIR}/../..)
ENDIF(WIN32)

# add all optional libraries to this variable
SET(optionalLibs)

# add all required libraries to this variable
SET(requiredLibs)

IF(!WIN32)
SET(requiredLibs ${requiredLibs} H3DUtil)
ENDIF(!WIN32)

INCLUDE_DIRECTORIES( ../include ${HAPI_SOURCE_DIR}/../../H3DUtil/include )

IF(WIN32)
  SET(requiredLibs ${requiredLibs} winmm.lib )
ENDIF(WIN32)

SET(CMAKE_MODULE_PATH ${HAPI_SOURCE_DIR}/modules )

FIND_PACKAGE(PTHREAD REQUIRED)
IF(PTHREAD_FOUND)
  INCLUDE_DIRECTORIES( ${PTHREAD_INCLUDE_DIR} ) 
  SET(requiredLibs ${requiredLibs} ${PTHREAD_LIBRARIES} )
ENDIF(PTHREAD_FOUND)

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
  SET(HAVE_DHDAPI 1)
  INCLUDE_DIRECTORIES( ${DHD_INCLUDE_DIR} ) 
  SET(optionalLibs ${optionalLibs} ${DHD_LIBRARIES} )
ENDIF(DHD_FOUND)

FIND_PACKAGE(FalconAPI)
IF(FALCONAPI_FOUND)
  SET(HAVE_FALCONAPI 1)
  INCLUDE_DIRECTORIES( ${FALCONAPI_INCLUDE_DIR} ) 
  SET(optionalLibs ${optionalLibs} ${FALCONAPI_LIBRARIES} )
ENDIF(FALCONAPI_FOUND)

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

# make sure symbols are exported.
SET_TARGET_PROPERTIES( HAPI PROPERTIES COMPILE_FLAGS "-DHAPI_EXPORTS" )

# add the libraries needed for linking
TARGET_LINK_LIBRARIES( HAPI ${requiredLibs} ${optionalLibs} )

# make the name of debug libraries end in _d.
SET_TARGET_PROPERTIES( HAPI PROPERTIES DEBUG_POSTFIX "_d" )

# set the version of the library
SET_TARGET_PROPERTIES( HAPI PROPERTIES VERSION ${HAPI_FULL_VERSION} )

# change the name depending on compiler to be able to tell them apart
# since they are not compatible with each other. 
IF(MSVC70 OR MSVC71)
  SET_TARGET_PROPERTIES( HAPI PROPERTIES OUTPUT_NAME HAPI_vc7 )
ELSEIF(MSVC80)
  SET_TARGET_PROPERTIES( HAPI PROPERTIES OUTPUT_NAME HAPI_vc8 )
ELSEIF(MSCV90)
  SET_TARGET_PROPERTIES( HAPI PROPERTIES OUTPUT_NAME HAPI_vc9 )
ENDIF(MSVC70 OR MSVC71)

IF( WIN32 )
  TARGET_LINK_LIBRARIES( OpenHapticsRenderer ${requiredLibs} ${optionalLibs} )
  TARGET_LINK_LIBRARIES( Chai3DRenderer ${requiredLibs} ${optionalLibs} atls.lib )

  # make sure symbols are exported.       
  SET_TARGET_PROPERTIES( OpenHapticsRenderer 
                         PROPERTIES COMPILE_FLAGS
                         "-DOPENHAPTICSRENDERER_EXPORTS" )
  SET_TARGET_PROPERTIES( Chai3DRenderer 
                         PROPERTIES COMPILE_FLAGS "-DCHAI3DRENDERER_EXPORTS" )

  # make the name of debug libraries end in _d.
  SET_TARGET_PROPERTIES( OpenHapticsRenderer PROPERTIES DEBUG_POSTFIX "_d" )
  SET_TARGET_PROPERTIES( Chai3DRenderer PROPERTIES DEBUG_POSTFIX "_d" )

  # set the version of the library
  SET_TARGET_PROPERTIES( OpenHapticsRenderer PROPERTIES VERSION ${HAPI_FULL_VERSION} )   
  SET_TARGET_PROPERTIES( Chai3DRenderer PROPERTIES VERSION ${HAPI_FULL_VERSION} )   

  # change the name depending on compiler to be able to tell them apart
  # since they are not compatible with each other. 
  IF(MSVC70 OR MSVC71)
    SET_TARGET_PROPERTIES( OpenHapticsRenderer PROPERTIES OUTPUT_NAME OpenHapticsRenderer_vc7 )
  ELSEIF(MSVC80)
    SET_TARGET_PROPERTIES( OpenHapticsRenderer PROPERTIES OUTPUT_NAME OpenHapticsRenderer_vc8 )
  ELSEIF(MSCV90)
    SET_TARGET_PROPERTIES( OpenHapticsRenderer PROPERTIES OUTPUT_NAME OpenHapticsRenderer_vc9 )
  ENDIF(MSVC70 OR MSVC71)

  IF(MSVC70 OR MSVC71)
    SET_TARGET_PROPERTIES( Chai3DRenderer PROPERTIES OUTPUT_NAME Chai3DRenderer_vc7 )
  ELSEIF(MSVC80)
    SET_TARGET_PROPERTIES( Chai3DRenderer PROPERTIES OUTPUT_NAME Chai3DRenderer_vc8 )
  ELSEIF(MSCV90)
    SET_TARGET_PROPERTIES( Chai3DRenderer PROPERTIES OUTPUT_NAME Chai3DRenderer_vc9 )
  ENDIF(MSVC70 OR MSVC71)
ENDIF( WIN32 )

# autogenerate HAPI.h depending on the libraries available.
CONFIGURE_FILE( ../include/HAPI/HAPI.cmake ../include/HAPI/HAPI.h )

SET( CMAKE_INSTALL_PREFIX ${HAPI_SOURCE_DIR}/../.. )

INSTALL( TARGETS HAPI 
         LIBRARY DESTINATION lib
         RUNTIME DESTINATION bin
         ARCHIVE DESTINATION lib )

IF(WIN32)
  INSTALL( TARGETS OpenHapticsRenderer
           LIBRARY DESTINATION lib
           RUNTIME DESTINATION bin
           ARCHIVE DESTINATION lib )

  INSTALL( TARGETS Chai3DRenderer 
           LIBRARY DESTINATION lib
           RUNTIME DESTINATION bin
           ARCHIVE DESTINATION lib )
ELSE(WIN32)
  INSTALL( DIRECTORY ../include/HAPI ../OpenHapticsRenderer/include/HAPI ../Chai3DRenderer/include/HAPI
           DESTINATION /usr/local/include 
           PATTERN .svn EXCLUDE )
ENDIF(WIN32)    

# include H3DUtil in the build
ADD_SUBDIRECTORY( ${HAPI_SOURCE_DIR}/../../H3DUtil/build
                  ${HAPI_SOURCE_DIR}/../../H3DUtil/build )

# HAPI is dependent on H3DUtil
ADD_DEPENDENCIES( HAPI H3DUtil )

IF(WIN32)
  ADD_DEPENDENCIES( OpenHapticsRenderer HAPI )
  ADD_DEPENDENCIES( OpenHapticsRenderer H3DUtil )
  ADD_DEPENDENCIES( Chai3DRenderer HAPI )
  ADD_DEPENDENCIES( Chai3DRenderer H3DUtil )
ENDIF(WIN32)