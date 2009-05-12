IF(WIN32)
  cmake_minimum_required(VERSION 2.6.0)
ENDIF(WIN32)
# Make include files in paths like /usr/local/include have preference before
# files in OSX frameworks. 
SET(CMAKE_FIND_FRAMEWORK "LAST")
PROJECT (HAPI)

# Add a cache variable GENERATE_CPACK_PROJECT to have the choice of generating a project
# for packaging HAPI. Default is NO since most people will not use this.
IF( NOT DEFINED GENERATE_CPACK_PROJECT )
  SET( GENERATE_CPACK_PROJECT "NO" CACHE BOOL "Decides if a cpack project should be generated. The project in the first loaded CMakeLists will configure CPack." )
  MARK_AS_ADVANCED(GENERATE_CPACK_PROJECT)
ENDIF( NOT DEFINED GENERATE_CPACK_PROJECT )

# Add a cache variable PREFER_STATIC_LIBRARIES to have the choice of generating a project
# linking against static libraries if they exist. Default is NO since most people will not use this.
IF( NOT DEFINED PREFER_STATIC_LIBRARIES )
  SET( PREFER_STATIC_LIBRARIES "NO" CACHE BOOL "Decides if CMake should prefer static libraries to dynamic libraries when both exist." )
  MARK_AS_ADVANCED(PREFER_STATIC_LIBRARIES)
ENDIF( NOT DEFINED PREFER_STATIC_LIBRARIES )

IF( PREFER_STATIC_LIBRARIES )
  SET( CMAKE_FIND_LIBRARY_SUFFIXES .a;${CMAKE_FIND_LIBRARY_SUFFIXES} )	
ENDIF( PREFER_STATIC_LIBRARIES )


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

# If cpack should be configured.
IF( GENERATE_CPACK_PROJECT )
  SET(CPACK_PACKAGE_DESCRIPTION_SUMMARY "HAPI. A cross platform, device independent haptics library.")
  SET(CPACK_PACKAGE_VENDOR "SenseGraphics AB")
  SET(CPACK_PACKAGE_CONTACT "support@sensegraphics.com" )
  SET(CPACK_PACKAGE_DESCRIPTION_FILE "${HAPI_SOURCE_DIR}/../ReadMe")
  SET(CPACK_RESOURCE_FILE_LICENSE "${HAPI_SOURCE_DIR}/../LICENSE")
  SET(CPACK_INSTALL_CMAKE_PROJECTS "${CMAKE_CURRENT_BINARY_DIR};HAPI;ALL;/" )
  SET(CPACK_PACKAGE_INSTALL_DIRECTORY "HAPI" )

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
                                  /build/vc9
                                  /build/vc8
                                  /build/vc7
                                  /osx/
                                  /doc/H3DAPI/ )
  
  SET(CPACK_PACKAGE_VERSION_MAJOR ${HAPI_MAJOR_VERSION})
  SET(CPACK_PACKAGE_VERSION_MINOR ${HAPI_MINOR_VERSION})
  SET(CPACK_PACKAGE_VERSION_PATCH ${HAPI_BUILD_VERSION})

  IF(WIN32 AND NOT UNIX)
    # Cached variable used by H3DAPIs GENERATE_CPACK_PROJECT to get the correct installation directories.
    SET( HAPI_CPACK_INSTALLED_DIRECTORIES ""
        CACHE INTERNAL "Installed directories of HAPI." )
    SET( CPACK_PACKAGE_START_MENU_NAME "HAPI 1.0" )

    SET( HAPIExamples_cmake_install "" CACHE PATH "Needs to be set if examples for HAPI is desired to be packed in the install. Set to location of cmake_install.cmake for the examples." )
    MARK_AS_ADVANCED(HAPIExamples_cmake_install)
    IF( HAPIExamples_cmake_install )
      SET(CPACK_INSTALL_CMAKE_PROJECTS ${CPACK_INSTALL_CMAKE_PROJECTS}
                                       "${HAPIExamples_cmake_install};Examples;ALL;/" )
      SET( CPACK_ADD_HAPIEXAMPLES_LINKS "ON" )
      
      FIND_PACKAGE(wxWidgetsWin)
    ELSE( HAPIExamples_cmake_install )
      SET(HAPI_CPACK_IGNORE_PATTERNS ${HAPI_CPACK_IGNORE_PATTERNS}
                                       /External/include/wx )
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
                           /xerces-c_2_8.dll\\\\.dll$
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
                           /Xerces-c_static_2\\\\.lib$
                           /include/3dconnexion
                           /include/AL
                           /include/Cg
                           /include/curl
                           /include/DirectShow
                           /include/fparser
                           /include/freetype
                           /include/FTGL
                           /include/libaudiofile
                           /include/ogg
                           /include/teem
                           /include/vorbis
                           /include/xercesc
                           /HAPI/doc/
                           /H3DUtil/doc/
                           /HapticAPI2
                           /ftgl_static_MTD_d\\\\.lib$
                           /ftgl_static_MTD_vc8\\\\.lib$
                           /ftgl_static_MTD_vc8_d\\\\.lib$
                           /ftgl_static_MTD_vc9\\\\.lib$
                           /ftgl_static_MTD_vc9_d\\\\.lib$
                           /Xerces-c_static_2_d\\\\.lib$
                           /xerces-c_static_2_vc8\\\\.lib$
                           /xerces-c_static_2_vc8_d\\\\.lib$
                           /xerces-c_static_2_vc9\\\\.lib$
                           /xerces-c_static_2_vc9_d\\\\.lib$
                           /dcmdata\\\\.lib$
                           /dcmimage\\\\.lib$
                           /dcmimgle\\\\.lib$
                           /z\\\\.lib$
                           /png\\\\.lib$
                           /bz2\\\\.lib$
                           /ofstd\\\\.lib$
                           /Definitions.h$ )
    SET(CPACK_INSTALLED_DIRECTORIES ${CPACK_INSTALLED_DIRECTORIES}
                                    "${HAPI_SOURCE_DIR}/../../HAPI;HAPI"
                                    "${HAPI_SOURCE_DIR}/../../H3DUtil;H3DUtil"
                                    "${HAPI_SOURCE_DIR}/../../doc;doc"
                                    "${HAPI_SOURCE_DIR}/../../bin;bin"
                                    "${HAPI_SOURCE_DIR}/../../lib;lib" )
    
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

# add all optional libraries to this variable
SET(optionalLibs)

# add all required libraries to this variable
SET(requiredLibs)

FIND_PACKAGE(PTHREAD REQUIRED)
IF(PTHREAD_FOUND)
  INCLUDE_DIRECTORIES( ${PTHREAD_INCLUDE_DIR} ) 
  SET(requiredLibs ${requiredLibs} ${PTHREAD_LIBRARIES} )
ENDIF(PTHREAD_FOUND)

FIND_PACKAGE(OpenHaptics)
IF(OPENHAPTICS_FOUND)
  SET(HAVE_OPENHAPTICS 1)
  INCLUDE_DIRECTORIES( ${OPENHAPTICS_INCLUDE_DIR} 
                       ../OpenHapticsRenderer/include )
  SET(optionalLibs ${optionalLibs} ${OPENHAPTICS_LIBRARIES} )
ELSEIF( NOT WIN32 )
  INCLUDE_DIRECTORIES( ../OpenHapticsRenderer/include )
ENDIF(OPENHAPTICS_FOUND)

FIND_PACKAGE(Chai3D)
IF(CHAI3D_FOUND)
  SET(HAVE_CHAI3D 1)
  INCLUDE_DIRECTORIES( ${CHAI3D_INCLUDE_DIR}
                       ../Chai3DRenderer/include )
  SET(optionalLibs ${optionalLibs} ${CHAI3D_LIBRARIES} )
ELSEIF( NOT WIN32 )
  INCLUDE_DIRECTORIES( ../Chai3DRenderer/include )
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

FIND_PACKAGE(SimballMedical)
IF(SIMBALLMEDICAL_FOUND)
  SET(HAVE_SIMBALLMEDICAL_API 1)
  INCLUDE_DIRECTORIES( ${SIMBALLMEDICAL_INCLUDE_DIR} )
  SET(optionalLibs ${optionalLibs} ${SIMBALLMEDICAL_LIBRARIES} )
ENDIF(SIMBALLMEDICAL_FOUND)

FIND_PACKAGE(OpenGL REQUIRED)
IF(OPENGL_FOUND)
  SET(HAVE_OPENGL 1)
  INCLUDE_DIRECTORIES( ${OPENGL_INCLUDE_DIR} )
  SET(requiredLibs ${requiredLibs} ${OPENGL_LIBRARIES} )
ENDIF(OPENGL_FOUND)

# The current implementation only have support for Haptic Master on Windows.
# No use in looking for the libs/dlls on the system since the dlls for
# haptic master are run-time loaded.
IF(WIN32)
  SET(HAVE_HAPTIC_MASTER_API 1)
ENDIF(WIN32)

# add all sources, they are added to variables called HAPI_SRCS,
# OH_SRCS and CHAI_SRCS defined in the included file. Header files are added
# to variables named HAPI_HEADERS, OH_HEADERS and CHAIR_HEADERS.
INCLUDE( ${HAPI_SOURCE_DIR}/HAPISourceFiles.txt )

# add the HAPI.rc resource file if Visual Studio
IF(MSVC)
  SET( HAPI_SRCS ${HAPI_SRCS} ${CMAKE_CURRENT_BINARY_DIR}/HAPI.rc )
ENDIF(MSVC)

# If cpack project should be generated and windows is the system on which to generate
# a number of extra directories needs to be added. They are added to an internal cached
# variable which can be used by other cmakelists that use HAPI as subdirectory (e.g. H3DAPI).
IF( GENERATE_CPACK_PROJECT )
  
  IF( WIN32 )
    IF( PTHREAD_FOUND )
      SET( HAPI_CPACK_INSTALLED_DIRECTORIES ${HAPI_CPACK_INSTALLED_DIRECTORIES}
                                            "${PTHREAD_INCLUDE_DIR}/../../include;External/include"
           CACHE INTERNAL "Installed directories of HAPI." )
    ENDIF( PTHREAD_FOUND )
    
    SET( HAPI_CPACK_INSTALLED_DIRECTORIES ${HAPI_CPACK_INSTALLED_DIRECTORIES}
                                          "${HAPI_SOURCE_DIR}/../../External/bin;External/bin"
         CACHE INTERNAL "Installed directories of HAPI." )  
    SET( HAPI_CPACK_INSTALLED_DIRECTORIES ${HAPI_CPACK_INSTALLED_DIRECTORIES}
                                          "${HAPI_SOURCE_DIR}/../../External/lib;External/lib"
         CACHE INTERNAL "Installed directories of HAPI." )
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
  ADD_LIBRARY(HAPI SHARED ${HAPI_SRCS} ${HAPI_HEADERS} )
  IF(OPENHAPTICS_FOUND)
    IF(MSVC)
      SET( OH_SRCS ${OH_SRCS} ${CMAKE_CURRENT_BINARY_DIR}/OpenHapticsRenderer.rc )
    ENDIF(MSVC)
    ADD_LIBRARY(OpenHapticsRenderer SHARED ${OH_SRCS} ${OH_HEADERS} )
  ENDIF(OPENHAPTICS_FOUND)
  
  IF(CHAI3D_FOUND)
    IF(MSVC)
      SET( CHAI_SRCS ${CHAI_SRCS} ${CMAKE_CURRENT_BINARY_DIR}/Chai3DRenderer.rc )
    ENDIF(MSVC)
    ADD_LIBRARY(Chai3DRenderer SHARED ${CHAI_SRCS} ${CHAI_HEADERS} )
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
  IF( CMAKE_INSTALL_PREFIX_INITIALIZED_TO_DEFAULT )
    SET( CMAKE_INSTALL_PREFIX ${HAPI_SOURCE_DIR}/../.. CACHE PATH "Install path prefix, prepended onto install directories." FORCE )
  ENDIF( CMAKE_INSTALL_PREFIX_INITIALIZED_TO_DEFAULT )
 
  # is this only for msvc or for all windows compilers?
  SET(requiredLibs ${requiredLibs} winmm.lib DelayImp.lib )
ELSE(WIN32)
  ADD_LIBRARY(HAPI SHARED ${HAPI_SRCS} ${OH_SRCS} ${CHAI_SRCS}
                          ${HAPI_HEADERS} ${OH_HEADERS} ${CHAI_HEADERS} )
ENDIF(WIN32)

# Needed to link correctly on MSVC70 and MSVC71 because the dependency is not enough
# to generate correct project files with this version of CMake.
IF(NOT WIN32)
  SET(requiredLibs ${requiredLibs} optimized H3DUtil debug H3DUtil_d )
ELSEIF(MSVC70 OR MSVC71)
  SET(requiredLibs ${requiredLibs} H3DUtil )
ENDIF(NOT WIN32)

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
  INSTALL( FILES ${HAPI_HEADERS} ${OH_HEADERS} ${CHAI_HEADERS}
           DESTINATION include/HAPI )
ENDIF(WIN32)


IF( NOT DEFINED HAPI_EXAMPLE_PROJECTS )
  SET( HAPI_EXAMPLE_PROJECTS "NO" CACHE BOOL "If set to YES the example projects distributed with HAPI will be included in the build." )
ENDIF( NOT DEFINED HAPI_EXAMPLE_PROJECTS )

IF( HAPI_EXAMPLE_PROJECTS )
  # This is done to avoid placing everything directly in "build" catalogue if some other catalogue is chosen.
  STRING( REPLACE ${HAPI_SOURCE_DIR} ${HAPI_SOURCE_DIR}/../examples HAPI_EXAMPLES_BINARY_DIR ${CMAKE_CURRENT_BINARY_DIR} )
  ADD_SUBDIRECTORY( ${HAPI_SOURCE_DIR}/../examples
                    ${HAPI_EXAMPLES_BINARY_DIR} )
  IF( DEFINED DirectXExample_SOURCE_DIR )
    ADD_DEPENDENCIES( DirectXExample HAPI )
  ENDIF( DEFINED DirectXExample_SOURCE_DIR )
  
  IF( DEFINED FeedbackBufferCollector_SOURCE_DIR )
    ADD_DEPENDENCIES( FeedbackBufferCollector HAPI )
  ENDIF( DEFINED FeedbackBufferCollector_SOURCE_DIR )
  
  IF( DEFINED HAPIDemo_SOURCE_DIR )
    ADD_DEPENDENCIES( HAPIDemo HAPI )
  ENDIF( DEFINED HAPIDemo_SOURCE_DIR )
  
  IF( DEFINED SpringExample_SOURCE_DIR )
    ADD_DEPENDENCIES( SpringExample HAPI )
  ENDIF( DEFINED SpringExample_SOURCE_DIR )
  
  IF( DEFINED SurfaceExample_SOURCE_DIR )
    ADD_DEPENDENCIES( SurfaceExample HAPI )
  ENDIF( DEFINED SurfaceExample_SOURCE_DIR )
  
  IF( DEFINED ThreadExamples_SOURCE_DIR )
    IF( DEFINED H3DUtil_SOURCE_DIR )
      ADD_DEPENDENCIES( SimpleThreadPrint H3DUtil )
      ADD_DEPENDENCIES( SimpleThreadPrintLock H3DUtil )
      ADD_DEPENDENCIES( PeriodicThreadCallbacks H3DUtil )
    ELSE( DEFINED H3DUtil_SOURCE_DIR )
      ADD_DEPENDENCIES( SimpleThreadPrint HAPI )
      ADD_DEPENDENCIES( SimpleThreadPrintLock HAPI )
      ADD_DEPENDENCIES( PeriodicThreadCallbacks HAPI )
    ENDIF( DEFINED H3DUtil_SOURCE_DIR )
  ENDIF( DEFINED ThreadExamples_SOURCE_DIR )
ENDIF( HAPI_EXAMPLE_PROJECTS )

