PROJECT (HAPI)

# include H3DUtil in the build
# Should this part only be done for Windows? It is used to avoid placing everything directly in "build" catalogue if some other catalogue is chosen.
STRING( REPLACE ${HAPI_SOURCE_DIR} ${HAPI_SOURCE_DIR}/../../H3DUtil/build BINARY_DIR_FOR_H3DUTIL ${CMAKE_CURRENT_BINARY_DIR} )
ADD_SUBDIRECTORY( ${HAPI_SOURCE_DIR}/../../H3DUtil/build
                  ${BINARY_DIR_FOR_H3DUTIL} )

SET( HAPI_MAJOR_VERSION 1 )
SET( HAPI_MINOR_VERSION 0 )
SET( HAPI_BUILD_VERSION 0 )

SET(CPACK_PACKAGE_DESCRIPTION_SUMMARY "HAPI. A cross platform device independent haptics library.")
SET(CPACK_PACKAGE_VENDOR "SenseGraphics AB")
SET(CPACK_PACKAGE_DESCRIPTION_FILE "${HAPI_SOURCE_DIR}/../ReadMe.txt")
SET(CPACK_RESOURCE_FILE_LICENSE "${HAPI_SOURCE_DIR}/../LICENSE")

SET(CPACK_PACKAGE_VERSION_MAJOR ${HAPI_MAJOR_VERSION})
SET(CPACK_PACKAGE_VERSION_MINOR ${HAPI_MINOR_VERSION})
SET(CPACK_PACKAGE_VERSION_PATCH ${HAPI_BUILD_VERSION})
# TODO: fix different for windows and unix if needed.
IF(WIN32 AND NOT UNIX)
  SET(CPACK_IGNORE_FILES /\\\\.svn/
                         \\\\.obj$
                         \\\\.vcproj$
                         \\\\.ncb$
                         \\\\.log$
                         \\\\.suo$
                         \\\\.sln$
                         \\\\.zip$
                         \\\\.dir/
                         \\\\.user$
                         \\\\.cv$
                         /Debug
                         /debug
                         /Release
                         /release
                         /linux
                         /win32
                         /osx )
  SET(CPACK_INSTALLED_DIRECTORIES "${HAPI_SOURCE_DIR}/../../HAPI;HAPI"
                                  "${HAPI_SOURCE_DIR}/../../H3DUtil;H3DUtil" )
ENDIF(WIN32 AND NOT UNIX)
INCLUDE(CPack)


SET( HAPI_FULL_VERSION
${HAPI_MAJOR_VERSION}.${HAPI_MINOR_VERSION}.${HAPI_BUILD_VERSION} ) 

#add all sources
AUX_SOURCE_DIRECTORY(../src HAPI_SRCS)

# add the HAPI.rc resource file if Visual Studio
IF(MSVC)
  SET( HAPI_SRCS ${HAPI_SRCS} ${HAPI_SOURCE_DIR}/HAPI.rc )
ENDIF(MSVC)


AUX_SOURCE_DIRECTORY(../OpenHapticsRenderer/src OH_SRCS)
AUX_SOURCE_DIRECTORY(../Chai3DRenderer/src CHAI_SRCS)
INCLUDE_DIRECTORIES(../OpenHapticsRenderer/include
                    ../Chai3DRenderer/include )


# add all optional libraries to this variable
SET(optionalLibs)

# add all required libraries to this variable
SET(requiredLibs)

SET(CMAKE_MODULE_PATH ${HAPI_SOURCE_DIR}/modules )

FIND_PACKAGE(PTHREAD REQUIRED)
IF(PTHREAD_FOUND)
  INCLUDE_DIRECTORIES( ${PTHREAD_INCLUDE_DIR} ) 
  IF(WIN32)
    STRIP_AND_ADD_LIBRARY_DIRECTORIES( ${PTHREAD_LIBRARIES})
  ELSE(WIN32)
    SET(requiredLibs ${requiredLibs} ${PTHREAD_LIBRARIES} )
  ENDIF(WIN32)
ENDIF(PTHREAD_FOUND)

FIND_PACKAGE(OpenHaptics)
IF(OPENHAPTICS_FOUND)
  SET(HAVE_OPENHAPTICS 1)
  INCLUDE_DIRECTORIES( ${OPENHAPTICS_INCLUDE_DIR} )
  IF(WIN32)
    STRIP_AND_ADD_LIBRARY_DIRECTORIES( ${OPENHAPTICS_LIBRARIES} )
  ELSE(WIN32)
    SET(optionalLibs ${optionalLibs} ${OPENHAPTICS_LIBRARIES} )
  ENDIF(WIN32)
ENDIF(OPENHAPTICS_FOUND)

FIND_PACKAGE(Chai3D)
IF(CHAI3D_FOUND)
  SET(HAVE_CHAI3D 1)
  INCLUDE_DIRECTORIES( ${CHAI3D_INCLUDE_DIR} )
  IF(WIN32)
    STRIP_AND_ADD_LIBRARY_DIRECTORIES( ${CHAI3D_LIBRARIES} )
  ELSE(WIN32)
    SET(optionalLibs ${optionalLibs} ${CHAI3D_LIBRARIES} )
  ENDIF(WIN32)
ENDIF(CHAI3D_FOUND)

FIND_PACKAGE(DHD)
IF(DHD_FOUND)
  SET(HAVE_DHDAPI 1)
  INCLUDE_DIRECTORIES( ${DHD_INCLUDE_DIR} )
  IF(WIN32)
    STRIP_AND_ADD_LIBRARY_DIRECTORIES( ${DHD_LIBRARIES} )
  ELSE(WIN32)
    SET(optionalLibs ${optionalLibs} ${DHD_LIBRARIES} )
  ENDIF(WIN32)
ENDIF(DHD_FOUND)

FIND_PACKAGE(FalconAPI)
IF(FALCONAPI_FOUND)
  SET(HAVE_FALCONAPI 1)
  INCLUDE_DIRECTORIES( ${FALCONAPI_INCLUDE_DIR} )
  IF(WIN32)
    STRIP_AND_ADD_LIBRARY_DIRECTORIES( ${FALCONAPI_LIBRARIES} )
  ELSE(WIN32)
    SET(optionalLibs ${optionalLibs} ${FALCONAPI_LIBRARIES} )
  ENDIF(WIN32)
ENDIF(FALCONAPI_FOUND)

FIND_PACKAGE(Haptik)
IF(HAPTIK_FOUND)
  SET(HAVE_HAPTIK_LIBRARY 1)
  INCLUDE_DIRECTORIES( ${HAPTIK_INCLUDE_DIR} )
  IF(WIN32)
    STRIP_AND_ADD_LIBRARY_DIRECTORIES( ${HAPTIK_LIBRARIES} )
  ELSE(WIN32)
    SET(optionalLibs ${optionalLibs} ${HAPTIK_LIBRARIES} )
  ENDIF(WIN32)
ENDIF(HAPTIK_FOUND)

FIND_PACKAGE(OpenGL REQUIRED)
IF(OPENGL_FOUND)
  SET(HAVE_OPENGL 1)
  INCLUDE_DIRECTORIES( ${OPENGL_INCLUDE_DIR} )
  IF(WIN32)
    STRIP_AND_ADD_LIBRARY_DIRECTORIES( ${OPENGL_LIBRARIES} )
  ELSE(WIN32)
    SET(requiredLibs ${requiredLibs} ${OPENGL_LIBRARIES} )
  ENDIF(WIN32)
ENDIF(OPENGL_FOUND)


# OpenHapticsRenderer and Chai3DRenderer are compiled as separate dlls 
# on Windows 
IF(WIN32)
  ADD_LIBRARY(HAPI SHARED ${HAPI_SRCS})
  IF(MSVC)
    SET( OH_SRCS ${OH_SRCS} ${HAPI_SOURCE_DIR}/OpenHapticsRenderer.rc )
    SET( CHAI_SRCS ${CHAI_SRCS} ${HAPI_SOURCE_DIR}/Chai3DRenderer.rc )
  ENDIF(MSVC)
  ADD_LIBRARY(OpenHapticsRenderer SHARED ${OH_SRCS})
  ADD_LIBRARY(Chai3DRenderer SHARED ${CHAI_SRCS})
ELSE(WIN32)
  ADD_LIBRARY(HAPI SHARED ${HAPI_SRCS} ${OH_SRCS} ${CHAI_SRCS} )
ENDIF(WIN32)

# make sure that the HAPI.rc contains the correct svn-version
IF( MSVC )
  EXECUTE_PROCESS( COMMAND ${HAPI_SOURCE_DIR}/FindSVNVersion ${HAPI_SOURCE_DIR}/../src/.svn/entries 
                   OUTPUT_VARIABLE HAPI_SVN_VERSION )
  # autogenerate resource files depending on the version
  CONFIGURE_FILE( HAPI.rc.cmake ${HAPI_SOURCE_DIR}/HAPI.rc )
  CONFIGURE_FILE( OpenHapticsRenderer.rc.cmake ${HAPI_SOURCE_DIR}/OpenHapticsRenderer.rc )
  CONFIGURE_FILE( Chai3DRenderer.rc.cmake ${HAPI_SOURCE_DIR}/Chai3DRenderer.rc )
  
  ADD_CUSTOM_COMMAND( TARGET HAPI 
                      PRE_BUILD 
                      COMMAND ${HAPI_SOURCE_DIR}/UpdateResourceFile 
                      ARGS HAPI ${HAPI_SOURCE_DIR}/HAPI.rc HAPI.rc.cmake
                      ${HAPI_MAJOR_VERSION} ${HAPI_MINOR_VERSION}
                      ${HAPI_BUILD_VERSION}
                      ${HAPI_SOURCE_DIR}/../src/.svn/entries )

  ADD_CUSTOM_COMMAND( TARGET OpenHapticsRenderer
                      PRE_BUILD 
                      COMMAND ${HAPI_SOURCE_DIR}/UpdateResourceFile 
                      ARGS HAPI ${HAPI_SOURCE_DIR}/OpenHapticsRenderer.rc OpenHapticsRenderer.rc.cmake
                      ${HAPI_MAJOR_VERSION} ${HAPI_MINOR_VERSION}
                      ${HAPI_BUILD_VERSION}
                      ${HAPI_SOURCE_DIR}/../src/.svn/entries )

  ADD_CUSTOM_COMMAND( TARGET Chai3DRenderer
                      PRE_BUILD 
                      COMMAND ${HAPI_SOURCE_DIR}/UpdateResourceFile 
                      ARGS HAPI ${HAPI_SOURCE_DIR}/Chai3DRenderer.rc Chai3DRenderer.rc.cmake
                      ${HAPI_MAJOR_VERSION} ${HAPI_MINOR_VERSION}
                      ${HAPI_BUILD_VERSION}
                      ${HAPI_SOURCE_DIR}/../src/.svn/entries )
ENDIF(MSVC)

# set the install directory to the H3D directory on Windows
IF(WIN32) 
 SET( CMAKE_INSTALL_PREFIX ${HAPI_SOURCE_DIR}/../..)
ENDIF(WIN32)

IF(NOT WIN32)
SET(requiredLibs ${requiredLibs} H3DUtil)
ENDIF(NOT WIN32)

INCLUDE_DIRECTORIES(  ${HAPI_SOURCE_DIR}/../include ${HAPI_SOURCE_DIR}/../../H3DUtil/include )

IF(WIN32)
  SET(requiredLibs ${requiredLibs} winmm.lib DelayImp.lib )
ENDIF(WIN32)


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
  
  # delay load information, not working for some reason when the / is there. For version 2.4.8 of cmake
  # According to some information it should work with latest trunk of cmake.
  # It is also not really correct in the fact that it does not add the choice in the properties
  # of the microsoft visual studio target but rather put it on the command prompt.
  # Only tested with microsoft visual studio c++ 2003 and 2005 express.
  SET_TARGET_PROPERTIES( OpenHapticsRenderer HAPI
                         PROPERTIES
                         LINK_FLAGS "/DELAYLOAD:\"HD.dll\"" )
  
  SET_TARGET_PROPERTIES( HAPI
                         PROPERTIES
                         LINK_FLAGS "/DELAYLOAD:\"dhd.dll\""
                         LINK_FLAGS "/DELAYLOAD:\"hdl.dll\""
                         LINK_FLAGS "/DELAYLOAD:\"Haptik.Library.dll\"" )

ENDIF( WIN32 )

# autogenerate HAPI.h depending on the libraries available.
CONFIGURE_FILE( ${HAPI_SOURCE_DIR}/../include/HAPI/HAPI.cmake ${HAPI_SOURCE_DIR}/../include/HAPI/HAPI.h )

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

# HAPI is dependent on H3DUtil
ADD_DEPENDENCIES( HAPI H3DUtil )

IF(WIN32)
  ADD_DEPENDENCIES( OpenHapticsRenderer HAPI )
  ADD_DEPENDENCIES( OpenHapticsRenderer H3DUtil )
  ADD_DEPENDENCIES( Chai3DRenderer HAPI )
  ADD_DEPENDENCIES( Chai3DRenderer H3DUtil )
ENDIF(WIN32)
