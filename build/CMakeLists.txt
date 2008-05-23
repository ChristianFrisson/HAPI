IF(WIN32)
  cmake_minimum_required(VERSION 2.6.0)
ENDIF(WIN32)
PROJECT (HAPI)

# Add a cache variable GENERATE_CPACK_PROJECT to have the choice of generating a project
# for packaging HAPI. Default is NO since most people will not use this.
IF( NOT DEFINED GENERATE_CPACK_PROJECT )
  SET( GENERATE_CPACK_PROJECT "NO" CACHE BOOL "Decides if a cpack project should be generated. The project in the first loaded CMakeLists will configure CPack." )
  MARK_AS_ADVANCED(GENERATE_CPACK_PROJECT)
ENDIF( NOT DEFINED GENERATE_CPACK_PROJECT )

# include H3DUtil in the build
IF( NOT GENERATE_CPACK_PROJECT OR WIN32 )
  # Should this part only be done for Windows? It is used to avoid placing everything directly in "build" catalogue if some other catalogue is chosen.
  STRING( REPLACE ${HAPI_SOURCE_DIR} ${HAPI_SOURCE_DIR}/../../H3DUtil/build BINARY_DIR_FOR_H3DUTIL ${CMAKE_CURRENT_BINARY_DIR} )
  STRING( COMPARE EQUAL ${BINARY_DIR_FOR_H3DUTIL} ${CMAKE_CURRENT_BINARY_DIR} SPECIAL_BINARY_DIR )
  IF( SPECIAL_BINARY_DIR )
    SET( BINARY_DIR_FOR_H3DUTIL ${HAPI_SOURCE_DIR}/../../H3DUtil/build )
  ENDIF( SPECIAL_BINARY_DIR )
  ADD_SUBDIRECTORY( ${HAPI_SOURCE_DIR}/../../H3DUtil/build
                    ${BINARY_DIR_FOR_H3DUTIL} )
ENDIF( NOT GENERATE_CPACK_PROJECT OR WIN32 )

SET( HAPI_MAJOR_VERSION 1 )
SET( HAPI_MINOR_VERSION 0 )
SET( HAPI_BUILD_VERSION 0 )

# Set here already in case a special NSIS template needs to be used by cpack.
SET(CMAKE_MODULE_PATH ${HAPI_SOURCE_DIR}/modules )
INCLUDE( StripAndAddLibraryDirectories )

# If cpack should be configured.
IF( GENERATE_CPACK_PROJECT )
  SET(CPACK_PACKAGE_DESCRIPTION_SUMMARY "HAPI. A cross platform, device independent haptics library.")
  SET(CPACK_PACKAGE_VENDOR "SenseGraphics AB")
  SET(CPACK_PACKAGE_CONTACT "support@sensegraphics.com" )
  SET(CPACK_PACKAGE_DESCRIPTION_FILE "${HAPI_SOURCE_DIR}/../ReadMe")
  SET(CPACK_RESOURCE_FILE_LICENSE "${HAPI_SOURCE_DIR}/../LICENSE")
  SET(CPACK_INSTALL_CMAKE_PROJECTS "${CMAKE_CURRENT_BINARY_DIR};HAPI;ALL;/" )
  SET(CPACK_PACKAGE_INSTALL_DIRECTORY "H3D" )

  # Our project depends on these debian packages for Linux.
  SET(DEBIAN_PACKAGE_DEPENDS "libgl-dev, h3dutil(>=1.0.0)" )

  # File patterns to ignore, common for all operating systems.
  SET( HAPI_CPACK_IGNORE_PATTERNS /\\\\.svn/
                                  \\\\.obj$
                                  \\\\.ncb$
                                  \\\\.log$
                                  \\\\.suo$
                                  \\\\.zip$
                                  \\\\.dir/
                                  \\\\.user$
                                  \\\\.cv$
                                  "/Debug(.)*/"
                                  "/debug(.)*/"
                                  /Release
                                  /release
                                  /linux
                                  /build/win32/
                                  /build/vc8
                                  /build/vc7
                                  /osx/
                                  /doc/H3DAPI/ )
  
  SET(CPACK_PACKAGE_VERSION_MAJOR ${HAPI_MAJOR_VERSION})
  SET(CPACK_PACKAGE_VERSION_MINOR ${HAPI_MINOR_VERSION})
  SET(CPACK_PACKAGE_VERSION_PATCH ${HAPI_BUILD_VERSION})
  # TODO: fix different for windows and unix if needed.
  IF(WIN32 AND NOT UNIX)
    # Cached variable used by H3DAPIs GENERATE_CPACK_PROJECT to get the correct installation directories.
    SET( HAPI_CPACK_INSTALLED_DIRECTORIES ""
        CACHE INTERNAL "Installed directories of HAPI." )

    SET( HAPIExamples_cmake_install "" CACHE PATH "Needs to be set if examples for HAPI is desired to be packed in the install. Set to location of cmake_install.cmake for the examples." )
    MARK_AS_ADVANCED(HAPIExamples_cmake_install)
    IF( HAPIExamples_cmake_install )
      SET(CPACK_INSTALL_CMAKE_PROJECTS ${CPACK_INSTALL_CMAKE_PROJECTS}
                                       "${HAPIExamples_cmake_install};Examples;ALL;/" )
      SET( CPACK_ADD_HAPIEXAMPLES_LINKS "ON" )
      
      FIND_PACKAGE(wxWidgetsWin)
      IF(wxWidgets_FOUND)
        SET( HAPI_CPACK_INSTALLED_DIRECTORIES ${HAPI_CPACK_INSTALLED_DIRECTORIES}
                                              "${wxWidgets_INCLUDE_DIR}/wx;External/include/wx"
           CACHE INTERNAL "Installed directories of HAPI." )
      ENDIF(wxWidgets_FOUND)
    ENDIF( HAPIExamples_cmake_install )  

    # This might be cleaned up a bit after cleaning External.
    SET(CPACK_IGNORE_FILES ${HAPI_CPACK_IGNORE_PATTERNS}
                           /fparser\\\\.lib$
                           /teem\\\\.lib$
                           /wxbase28_net\\\\.lib$
                           /wxbase28_odbc\\\\.lib$
                           /wxbase28_xml\\\\.lib$
                           /wxbase28_net\\\\.lib$
                           /wxbase28_odbc\\\\.lib$
                           /wxbase28_xml\\\\.lib$
                           /wxexpat\\\\.lib$
                           /wxjpeg\\\\.lib$
                           /wxmsw28_aui\\\\.lib$
                           /wxmsw28_dbgrid\\\\.lib$
                           /wxmsw28_html\\\\.lib$
                           /wxmsw28_media\\\\.lib$
                           /wxmsw28_qa\\\\.lib$
                           /wxmsw28_richtext\\\\.lib$
                           /wxmsw28_xrc\\\\.lib$
                           /wxpng\\\\.lib$
                           /wxregex\\\\.lib$
                           /wxtiff\\\\.lib$
                           /wxzlib\\\\.lib$
                           /pthreadGC2\\\\.dll$
                           /pthreadGCE2\\\\.dll$
                           /pthreadVSE2\\\\.dll$
                           /audiofile\\\\.dll$
                           /cg\\\\.dll$
                           /cgGL\\\\.dll$
                           /ftgl_dynamic_MTD\\\\.dll$
                           /libcurl\\\\.dll$
                           /ogg\\\\.dll$
                           /ogg_d\\\\.dll$
                           /OpenAL32\\\\.dll$
                           /vorbis\\\\.dll$
                           /vorbisfile\\\\.dll$
                           /wrap_oal\\\\.dll$
                           /xerces-c_2_7\\\\.dll$
                           /xerces-c_2_7D\\\\.dll$
                           /ALut\\\\.lib$
                           /audiofile\\\\.lib$
                           /cg\\\\.lib$
                           /cgGL\\\\.lib$
                           /freetype219MT\\\\.lib$
                           /freetype235\\\\.lib$
                           /ftgl_dynamic_MTD\\\\.lib$
                           /ftgl_static_MTD\\\\.lib$
                           /libaudiofile\\\\.lib$
                           /libcurl\\\\.lib$
                           /libcurl_static\\\\.lib$
                           /ogg\\\\.lib$
                           /ogg_static\\\\.lib$
                           /OpenAL32\\\\.lib$
                           /siapp\\\\.lib$
                           /spwmath\\\\.lib$
                           /strmbase\\\\.lib$
                           /vorbis\\\\.lib$
                           /vorbis_static\\\\.lib$
                           /vorbisfile\\\\.lib$
                           /vorbisfile_static\\\\.lib$
                           /xerces-c_2\\\\.lib$
                           /Xerces-c_static_2\\\\.lib$ )
    SET(CPACK_INSTALLED_DIRECTORIES ${CPACK_INSTALLED_DIRECTORIES}
                                    "${HAPI_SOURCE_DIR}/../../HAPI;HAPI"
                                    "${HAPI_SOURCE_DIR}/../../H3DUtil;H3DUtil"
                                    "${HAPI_SOURCE_DIR}/../../doc;doc" )
    
    #Extra links to start menu if values are "ON"
    SET( CPACK_ADD_HAPIDOC_LINKS "ON" )
    
    # Do not modify path since this is done by the NSIS template.
    SET( CPACK_NSIS_MODIFY_PATH "OFF" )
  ELSE(WIN32 AND NOT UNIX)
    SET(CPACK_SOURCE_IGNORE_FILES ${HAPI_CPACK_IGNORE_PATTERNS} )
    SET(CPACK_SOURCE_INSTALLED_DIRECTORIES "${HAPI_SOURCE_DIR}/../../HAPI;HAPI"
                                           "${HAPI_SOURCE_DIR}/../../H3DUtil;H3DUtil" )
  ENDIF(WIN32 AND NOT UNIX)
ENDIF( GENERATE_CPACK_PROJECT )

SET( HAPI_FULL_VERSION
${HAPI_MAJOR_VERSION}.${HAPI_MINOR_VERSION}.${HAPI_BUILD_VERSION} ) 

#add all sources
AUX_SOURCE_DIRECTORY(../src HAPI_SRCS)

# add the HAPI.rc resource file if Visual Studio
IF(MSVC)
  SET( HAPI_SRCS ${HAPI_SRCS} ${CMAKE_CURRENT_BINARY_DIR}/HAPI.rc )
ENDIF(MSVC)

# add all optional libraries to this variable
SET(optionalLibs)

# add all required libraries to this variable
SET(requiredLibs)

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
  AUX_SOURCE_DIRECTORY(../OpenHapticsRenderer/src OH_SRCS)
  INCLUDE_DIRECTORIES( ${OPENHAPTICS_INCLUDE_DIR} 
                       ../OpenHapticsRenderer/include )
  IF(WIN32)
    STRIP_AND_ADD_LIBRARY_DIRECTORIES( ${OPENHAPTICS_LIBRARIES} )
  ELSE(WIN32)
    SET(optionalLibs ${optionalLibs} ${OPENHAPTICS_LIBRARIES} )
  ENDIF(WIN32)
ELSEIF( NOT WIN32 )
  INCLUDE_DIRECTORIES( ../OpenHapticsRenderer/include )
ENDIF(OPENHAPTICS_FOUND)

FIND_PACKAGE(Chai3D)
IF(CHAI3D_FOUND)
  SET(HAVE_CHAI3D 1)
  AUX_SOURCE_DIRECTORY(../Chai3DRenderer/src CHAI_SRCS)
  INCLUDE_DIRECTORIES( ${CHAI3D_INCLUDE_DIR}
                       ../Chai3DRenderer/include )
  IF(WIN32)
    STRIP_AND_ADD_LIBRARY_DIRECTORIES( ${CHAI3D_LIBRARIES} )
  ELSE(WIN32)
    SET(optionalLibs ${optionalLibs} ${CHAI3D_LIBRARIES} )
  ENDIF(WIN32)
ELSEIF( NOT WIN32 )
  INCLUDE_DIRECTORIES( ../Chai3DRenderer/include )
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

FIND_PACKAGE(SimballMedical)
IF(SIMBALLMEDICAL_FOUND)
  SET(HAVE_SIMBALLMEDICAL_API 1)
  INCLUDE_DIRECTORIES( ${SIMBALLMEDICAL_INCLUDE_DIR} )
  IF(WIN32)
    STRIP_AND_ADD_LIBRARY_DIRECTORIES( ${SIMBALLMEDICAL_LIBRARIES} )
  ELSE(WIN32)
    SET(optionalLibs ${optionalLibs} ${SIMBALLMEDICAL_LIBRARIES} )
  ENDIF(WIN32)
ENDIF(SIMBALLMEDICAL_FOUND)

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

# If cpack project should be generated and windows is the system on which to generate
# a number of extra directories needs to be added. They are added to an internal cached
# variable which can be used by other cmakelists that use HAPI as subdirectory (e.g. H3DAPI).
IF( GENERATE_CPACK_PROJECT )
  
  IF( WIN32 )
  
    IF( PTHREAD_FOUND )
      SET( HAPI_CPACK_INSTALLED_DIRECTORIES ${HAPI_CPACK_INSTALLED_DIRECTORIES}
                                            "${PTHREAD_INCLUDE_DIR};External/include/pthread"
           CACHE INTERNAL "Installed directories of HAPI." )
    ENDIF( PTHREAD_FOUND )
  
    IF( CHAI3D_FOUND )
      SET( HAPI_CPACK_INSTALLED_DIRECTORIES ${HAPI_CPACK_INSTALLED_DIRECTORIES}
                                            "${CHAI3D_INCLUDE_DIR}/..;External/include/chai3d"
           CACHE INTERNAL "Installed directories of HAPI." )
    ENDIF( CHAI3D_FOUND )
  
    IF( DHD_FOUND )
      SET( HAPI_CPACK_INSTALLED_DIRECTORIES ${HAPI_CPACK_INSTALLED_DIRECTORIES}
                                            "${DHD_INCLUDE_DIR};External/include/DHD-API"
           CACHE INTERNAL "Installed directories of HAPI." )
    ENDIF( DHD_FOUND )

    IF( SIMBALLMEDICAL_FOUND )
      SET( HAPI_CPACK_INSTALLED_DIRECTORIES ${HAPI_CPACK_INSTALLED_DIRECTORIES}
                                            "${SIMBALLMEDICAL_INCLUDE_DIR}/Simball;External/include/Simball"
           CACHE INTERNAL "Installed directories of HAPI." )
    ENDIF( SIMBALLMEDICAL_FOUND )

    IF( OPENGL_FOUND )
      IF( OPENGL_INCLUDE_DIR )
        SET( HAPI_CPACK_INSTALLED_DIRECTORIES ${HAPI_CPACK_INSTALLED_DIRECTORIES}
                                              "${OPENGL_INCLUDE_DIR}/GL;External/include/GL"
             CACHE INTERNAL "Installed directories of HAPI." )
      ELSE( OPENGL_INCLUDE_DIR )
        SET( HAPI_CPACK_INSTALLED_DIRECTORIES ${HAPI_CPACK_INSTALLED_DIRECTORIES}
                                              "${HAPI_SOURCE_DIR}/../../External/include/GL;External/include/GL"
             CACHE INTERNAL "Installed directories of HAPI." )
      ENDIF( OPENGL_INCLUDE_DIR )
    ENDIF( OPENGL_FOUND )
    
    SET(CPACK_INSTALLED_DIRECTORIES ${CPACK_INSTALLED_DIRECTORIES}
                                    ${H3DUTIL_CPACK_INSTALLED_DIRECTORIES} )
    
    SET(CPACK_INSTALLED_DIRECTORIES ${CPACK_INSTALLED_DIRECTORIES}
                                    "${HAPI_SOURCE_DIR}/../../External/bin;External/bin" )  
    SET(CPACK_INSTALLED_DIRECTORIES ${CPACK_INSTALLED_DIRECTORIES}
                                    "${HAPI_SOURCE_DIR}/../../External/lib;External/lib" )
    
    SET( CPACK_INSTALLED_DIRECTORIES ${CPACK_INSTALLED_DIRECTORIES}
                                     ${HAPI_CPACK_INSTALLED_DIRECTORIES} )
  ENDIF( WIN32 )
  
  INCLUDE(CPack)
  INCLUDE(UseDebian)
  IF(DEBIAN_FOUND)
    ADD_DEBIAN_TARGETS(HAPI)
  ENDIF(DEBIAN_FOUND)
  
ENDIF( GENERATE_CPACK_PROJECT )

IF(WIN32)

  # OpenHapticsRenderer and Chai3DRenderer are compiled as separate dlls 
  # on Windows 
  ADD_LIBRARY(HAPI SHARED ${HAPI_SRCS})
  IF(OPENHAPTICS_FOUND)
    IF(MSVC)
      SET( OH_SRCS ${OH_SRCS} ${CMAKE_CURRENT_BINARY_DIR}/OpenHapticsRenderer.rc )
    ENDIF(MSVC)
    ADD_LIBRARY(OpenHapticsRenderer SHARED ${OH_SRCS})
  ENDIF(OPENHAPTICS_FOUND)
  
  IF(CHAI3D_FOUND)
    IF(MSVC)
      SET( CHAI_SRCS ${CHAI_SRCS} ${CMAKE_CURRENT_BINARY_DIR}/Chai3DRenderer.rc )
    ENDIF(MSVC)
    ADD_LIBRARY(Chai3DRenderer SHARED ${CHAI_SRCS})
  ENDIF(CHAI3D_FOUND)

  IF( MSVC )
    # make sure that HAPI.rc contains the correct svn-version
    EXECUTE_PROCESS( COMMAND ${HAPI_SOURCE_DIR}/FindSVNVersion ${HAPI_SOURCE_DIR}/../src/.svn/entries 
                     OUTPUT_VARIABLE HAPI_SVN_VERSION )

    ADD_CUSTOM_COMMAND( TARGET HAPI 
                        PRE_BUILD 
                        COMMAND ${HAPI_SOURCE_DIR}/UpdateResourceFile 
                        ARGS HAPI ${CMAKE_CURRENT_BINARY_DIR}/HAPI.rc HAPI.rc.cmake
                        ${HAPI_MAJOR_VERSION} ${HAPI_MINOR_VERSION}
                        ${HAPI_BUILD_VERSION}
                        ${HAPI_SOURCE_DIR}/../src/.svn/entries )

    # autogenerate resource files depending on the version
    CONFIGURE_FILE( HAPI.rc.cmake ${CMAKE_CURRENT_BINARY_DIR}/HAPI.rc )

    # make sure that HAPI.rc, OpenHapticsRenderer.rc and Chai3DRenderer.rc contains the correct svn-version
    # and autogenerate the resource files.
    IF(OPENHAPTICS_FOUND)
      CONFIGURE_FILE( OpenHapticsRenderer.rc.cmake ${CMAKE_CURRENT_BINARY_DIR}/OpenHapticsRenderer.rc )
      
      ADD_CUSTOM_COMMAND( TARGET OpenHapticsRenderer
                          PRE_BUILD 
                          COMMAND ${HAPI_SOURCE_DIR}/UpdateResourceFile 
                          ARGS HAPI ${CMAKE_CURRENT_BINARY_DIR}/OpenHapticsRenderer.rc OpenHapticsRenderer.rc.cmake
                          ${HAPI_MAJOR_VERSION} ${HAPI_MINOR_VERSION}
                          ${HAPI_BUILD_VERSION}
                          ${HAPI_SOURCE_DIR}/../src/.svn/entries )
    ENDIF(OPENHAPTICS_FOUND)

    IF(CHAI3D_FOUND)
      CONFIGURE_FILE( Chai3DRenderer.rc.cmake ${CMAKE_CURRENT_BINARY_DIR}/Chai3DRenderer.rc )
      
      ADD_CUSTOM_COMMAND( TARGET Chai3DRenderer
                          PRE_BUILD 
                          COMMAND ${HAPI_SOURCE_DIR}/UpdateResourceFile 
                          ARGS HAPI ${CMAKE_CURRENT_BINARY_DIR}/Chai3DRenderer.rc Chai3DRenderer.rc.cmake
                          ${HAPI_MAJOR_VERSION} ${HAPI_MINOR_VERSION}
                          ${HAPI_BUILD_VERSION}
                          ${HAPI_SOURCE_DIR}/../src/.svn/entries )
    ENDIF(CHAI3D_FOUND)
  ENDIF(MSVC)
  
  # set the install directory to the H3D directory on Windows
  SET( CMAKE_INSTALL_PREFIX ${HAPI_SOURCE_DIR}/../..)
 
  # is this only for msvc or for all windows compilers?
  SET(requiredLibs ${requiredLibs} winmm.lib DelayImp.lib )
ELSE(WIN32)
  ADD_LIBRARY(HAPI SHARED ${HAPI_SRCS} ${OH_SRCS} ${CHAI_SRCS} )
ENDIF(WIN32)

# Needed to link correctly on MSVC70 and MSVC71 because the dependency is not enough
# to generate correct project files with this version of CMake.
IF(NOT WIN32 OR MSVC70 OR MSVC71)
  SET(requiredLibs ${requiredLibs} H3DUtil)
ENDIF(NOT WIN32 OR MSVC70 OR MSVC71)

# Add the last include directories needed.
INCLUDE_DIRECTORIES(  ${HAPI_SOURCE_DIR}/../include ${HAPI_SOURCE_DIR}/../../H3DUtil/include )

# make sure symbols are exported.
SET( HAPI_COMPILE_FLAGS "-DHAPI_EXPORTS" )

# add the libraries needed for linking
TARGET_LINK_LIBRARIES( HAPI ${requiredLibs} ${optionalLibs} )

# make the name of debug libraries end in _d.
SET_TARGET_PROPERTIES( HAPI PROPERTIES DEBUG_POSTFIX "_d" )

# set the version of the library
SET_TARGET_PROPERTIES( HAPI PROPERTIES VERSION ${HAPI_FULL_VERSION} )

IF( WIN32 )
  
  # change the name depending on compiler to be able to tell them apart
  # since they are not compatible with each other. 
  IF(MSVC70 OR MSVC71)
    SET_TARGET_PROPERTIES( HAPI PROPERTIES OUTPUT_NAME HAPI_vc7 )
  ELSEIF(MSVC80)
    SET_TARGET_PROPERTIES( HAPI PROPERTIES OUTPUT_NAME HAPI_vc8 )
  ELSEIF(MSVC90)
    SET_TARGET_PROPERTIES( HAPI PROPERTIES OUTPUT_NAME HAPI_vc9 )
    SET( HAPI_COMPILE_FLAGS "${HAPI_COMPILE_FLAGS} /MP" )
  ENDIF(MSVC70 OR MSVC71)

  # Set properties for OpenHapticsRenderer if the project is created.
  IF(OPENHAPTICS_FOUND)
    TARGET_LINK_LIBRARIES( OpenHapticsRenderer ${requiredLibs} ${optionalLibs} HAPI )
    # make sure symbols are exported.
    SET( OpenHapticsRenderer_COMPILE_FLAGS "-DOPENHAPTICSRENDERER_EXPORTS" )
    
    # make the name of debug libraries end in _d.
    SET_TARGET_PROPERTIES( OpenHapticsRenderer PROPERTIES DEBUG_POSTFIX "_d" )
    # set the version of the library
    SET_TARGET_PROPERTIES( OpenHapticsRenderer PROPERTIES VERSION ${HAPI_FULL_VERSION} )
    
    # change the name depending on compiler to be able to tell them apart
    # since they are not compatible with each other. 
    IF(MSVC70 OR MSVC71)
      SET_TARGET_PROPERTIES( OpenHapticsRenderer PROPERTIES OUTPUT_NAME OpenHapticsRenderer_vc7 )
    ELSEIF(MSVC80)
      SET_TARGET_PROPERTIES( OpenHapticsRenderer PROPERTIES OUTPUT_NAME OpenHapticsRenderer_vc8 )
    ELSEIF(MSVC90)
      SET_TARGET_PROPERTIES( OpenHapticsRenderer PROPERTIES OUTPUT_NAME OpenHapticsRenderer_vc9 )
      SET( OpenHapticsRenderer_COMPILE_FLAGS "${OpenHapticsRenderer_COMPILE_FLAGS} /MP -D_CRT_SECURE_NO_DEPRECATE" )
    ENDIF(MSVC70 OR MSVC71)

    # Set compile flags
    SET_TARGET_PROPERTIES( OpenHapticsRenderer 
                           PROPERTIES COMPILE_FLAGS
                           "${OpenHapticsRenderer_COMPILE_FLAGS}" )

    # Set link flags
    SET_TARGET_PROPERTIES( OpenHapticsRenderer HAPI
                           PROPERTIES
                           LINK_FLAGS "/DELAYLOAD:\"HD.dll\"" )
    
    # When using OpenHaptics HAPI also need to delayload HD.dll.
    SET( HAPI_LINK_FLAGS "${HAPI_LINK_FLAGS} /DELAYLOAD:\"HD.dll\"" )
  ENDIF(OPENHAPTICS_FOUND)

  # Set properties for Chai3DRenderer if the project is created.
  IF(CHAI3D_FOUND)
    TARGET_LINK_LIBRARIES( Chai3DRenderer ${requiredLibs} ${optionalLibs} HAPI )
    # make sure symbols are exported.
    SET_TARGET_PROPERTIES( Chai3DRenderer 
                           PROPERTIES COMPILE_FLAGS "-DCHAI3DRENDERER_EXPORTS" )
    # make the name of debug libraries end in _d.
    SET_TARGET_PROPERTIES( Chai3DRenderer PROPERTIES DEBUG_POSTFIX "_d" )
    # set the version of the library
    SET_TARGET_PROPERTIES( Chai3DRenderer PROPERTIES VERSION ${HAPI_FULL_VERSION} )

    # change the name depending on compiler to be able to tell them apart
    # since they are not compatible with each other. 
    IF(MSVC70 OR MSVC71)
      SET_TARGET_PROPERTIES( Chai3DRenderer PROPERTIES OUTPUT_NAME Chai3DRenderer_vc7 )
    ELSEIF(MSVC80)
      SET_TARGET_PROPERTIES( Chai3DRenderer PROPERTIES OUTPUT_NAME Chai3DRenderer_vc8 )
    ELSEIF(MSVC90)
      SET_TARGET_PROPERTIES( Chai3DRenderer PROPERTIES OUTPUT_NAME Chai3DRenderer_vc9 )
    ENDIF(MSVC70 OR MSVC71)

    # Set link flags
    SET_TARGET_PROPERTIES( Chai3DRenderer
                           PROPERTIES
                           LINK_FLAGS_DEBUG "/NODEFAULTLIB:msvcrt" )
  ENDIF(CHAI3D_FOUND)

  # Set link flags for HAPI
  SET_TARGET_PROPERTIES( HAPI
                         PROPERTIES
                         LINK_FLAGS "${HAPI_LINK_FLAGS} /DELAYLOAD:\"dhd.dll\" /DELAYLOAD:\"hdl.dll\" /DELAYLOAD:\"Haptik.Library.dll\" /DELAYLOAD:\"SimballMedicalHID.dll\"" )
ENDIF( WIN32 )

# set compile flags for HAPI project
SET_TARGET_PROPERTIES( HAPI PROPERTIES COMPILE_FLAGS "${HAPI_COMPILE_FLAGS}" )

# autogenerate HAPI.h depending on the libraries available.
CONFIGURE_FILE( ${HAPI_SOURCE_DIR}/../include/HAPI/HAPI.cmake ${HAPI_SOURCE_DIR}/../include/HAPI/HAPI.h )

# Where to install HAPI
INSTALL( TARGETS HAPI 
         LIBRARY DESTINATION lib
         RUNTIME DESTINATION bin
         ARCHIVE DESTINATION lib )

# HAPI is dependent on H3DUtil
ADD_DEPENDENCIES( HAPI H3DUtil )

IF(WIN32)
  IF(OPENHAPTICS_FOUND)
    # Where to install OpenHapticsRenderer
    INSTALL( TARGETS OpenHapticsRenderer
             LIBRARY DESTINATION lib
             RUNTIME DESTINATION bin
             ARCHIVE DESTINATION lib )

    # OpenHapticsRenderer is dependent on HAPI and H3DUtil
    ADD_DEPENDENCIES( OpenHapticsRenderer HAPI )
    ADD_DEPENDENCIES( OpenHapticsRenderer H3DUtil )
  ENDIF(OPENHAPTICS_FOUND)

  IF(CHAI3D_FOUND)
    # Where to install Chai3DRenderer
    INSTALL( TARGETS Chai3DRenderer 
             LIBRARY DESTINATION lib
             RUNTIME DESTINATION bin
             ARCHIVE DESTINATION lib )

    # Chai3DRenderer is dependent on HAPI and H3DUtil
    ADD_DEPENDENCIES( Chai3DRenderer HAPI )
    ADD_DEPENDENCIES( Chai3DRenderer H3DUtil )
  ENDIF(CHAI3D_FOUND)
ELSE(WIN32)
  # Install header files on non-windows system (e.g. Unix).
  INSTALL( DIRECTORY ../include/HAPI ../OpenHapticsRenderer/include/HAPI ../Chai3DRenderer/include/HAPI
           DESTINATION include 
           PATTERN .svn EXCLUDE )
ENDIF(WIN32)
