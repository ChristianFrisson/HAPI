IF( COMMAND cmake_policy )
  IF( POLICY CMP0003 )
    cmake_policy(SET CMP0003 OLD)
  ENDIF( POLICY CMP0003 )
ENDIF( COMMAND cmake_policy )

IF(WIN32)
  cmake_minimum_required(VERSION 2.6.0)
ENDIF(WIN32)
# Make include files in paths like /usr/local/include have preference before
# files in OSX frameworks. 
SET(CMAKE_FIND_FRAMEWORK "LAST")

PROJECT (HAPI)

# set the install directory on Windows
IF( WIN32 AND CMAKE_INSTALL_PREFIX_INITIALIZED_TO_DEFAULT AND NOT H3D_CMAKE_INSTALL_PREFIX_ALREADY_SET )
  SET( CMAKE_INSTALL_PREFIX ${HAPI_SOURCE_DIR}/../.. CACHE PATH "Install path prefix, prepended onto install directories." FORCE )
  SET( H3D_CMAKE_INSTALL_PREFIX_ALREADY_SET TRUE )
ENDIF( WIN32 AND CMAKE_INSTALL_PREFIX_INITIALIZED_TO_DEFAULT AND NOT H3D_CMAKE_INSTALL_PREFIX_ALREADY_SET )

# Add a cache variable GENERATE_UNITY_BUILD to have the choice of selecting
# a unity build project. Default is NO.
IF( NOT DEFINED GENERATE_UNITY_BUILD )
  SET( GENERATE_UNITY_BUILD "NO" CACHE BOOL "Decides if a the generated project files should build through a unity build instead of a normal build. A unity builds packs all .cpp files into a UnityBuild.cpp file and then only include this in the project. This greatly reduces build times." )
ENDIF( NOT DEFINED GENERATE_UNITY_BUILD )

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
IF( ( WIN32 OR NOT H3D_USE_DEPENDENCIES_ONLY ) AND EXISTS "${HAPI_SOURCE_DIR}/../../H3DUtil/build" )
  MESSAGE( STATUS "Including H3DUtil" )
  ADD_SUBDIRECTORY( ${HAPI_SOURCE_DIR}/../../H3DUtil/build
                    ${CMAKE_CURRENT_BINARY_DIR}/H3DUtil )
  
  GET_DIRECTORY_PROPERTY( H3DUTIL_INCLUDE_DIR_TMP DIRECTORY ${HAPI_SOURCE_DIR}/../../H3DUtil/build INCLUDE_DIRECTORIES )
  SET( H3DUTIL_INCLUDE_DIR ${H3DUTIL_INCLUDE_DIR_TMP} CACHE INTERNAL "Set to internal so H3DUtil and/or HAPI can change it." )
ENDIF( ( WIN32 OR NOT H3D_USE_DEPENDENCIES_ONLY ) AND EXISTS "${HAPI_SOURCE_DIR}/../../H3DUtil/build" )

# Add the directory to INCLUDE_DIRECTORIES before any other statement to make sure that this header file is found first.
# This assumes that we always use "#include <path/include_header.h>" in our code even if the file is actually found in
# the same directory as the file that contains the include statement.
INCLUDE_DIRECTORIES( ${CMAKE_CURRENT_BINARY_DIR}/include )
SET( HAPI_INCLUDE_DIR ${CMAKE_CURRENT_BINARY_DIR}/include ${HAPI_SOURCE_DIR}/../include CACHE INTERNAL "Set to internal to propagate change" )

# include header directory.
INCLUDE_DIRECTORIES( ${HAPI_SOURCE_DIR}/../include )

SET( HAPI_MAJOR_VERSION 1 )
SET( HAPI_MINOR_VERSION 3 )
SET( HAPI_BUILD_VERSION 1 )

# Set here already in case a special NSIS template needs to be used by cpack.
SET(CMAKE_MODULE_PATH ${HAPI_SOURCE_DIR}/modules )
IF( COMMAND cmake_policy )
  if( POLICY CMP0011 )
    cmake_policy( SET CMP0011 NEW )
  endif( POLICY CMP0011 )
ENDIF( COMMAND cmake_policy )
INCLUDE( StripAndAddLibraryDirectories )

SET( HAPI_FULL_VERSION ${HAPI_MAJOR_VERSION}.${HAPI_MINOR_VERSION}.${HAPI_BUILD_VERSION} )

# add all optional libraries to this variable, only needed for HAPI project.
SET( HAPI_optionalLibs )

# add all required libraries to this variable, one for each project
SET( HAPI_requiredLibs)
SET( Chai3D_requiredLibs)
SET( OpenHaptics_requiredLibs)

IF( TARGET H3DUtil )
  # Target was created by ADD_SUBDIRECTORY command.
  INCLUDE_DIRECTORIES( ${H3DUTIL_INCLUDE_DIR} ) 
  SET( HAPI_requiredLibs ${HAPI_requiredLibs} H3DUtil )
ELSE( TARGET H3DUtil )
  #H3DUtil
  FIND_PACKAGE(H3DUtil REQUIRED)
  IF(H3DUTIL_FOUND)
    INCLUDE_DIRECTORIES( ${H3DUTIL_INCLUDE_DIR} ) 
    SET(HAPI_requiredLibs ${HAPI_requiredLibs} ${H3DUTIL_LIBRARIES} )
  ENDIF(H3DUTIL_FOUND)
ENDIF( TARGET H3DUtil )

FIND_PACKAGE(OpenHaptics)
IF(OPENHAPTICS_FOUND)
  SET(HAVE_OPENHAPTICS 1)
  INCLUDE_DIRECTORIES( ${OPENHAPTICS_INCLUDE_DIR} 
                       ../OpenHapticsRenderer/include )
	SET( HAPI_INCLUDE_DIR ${HAPI_INCLUDE_DIR} ${HAPI_SOURCE_DIR}/../OpenHapticsRenderer/include CACHE INTERNAL "Set to internal to propagate change" )
  SET(HAPI_optionalLibs ${HAPI_optionalLibs} ${OPENHAPTICS_LIBRARIES} )
  SET(OpenHaptics_requiredLibs ${OpenHaptics_requiredLibs} ${OPENHAPTICS_LIBRARIES} )
ELSEIF( NOT WIN32 )
  INCLUDE_DIRECTORIES( ../OpenHapticsRenderer/include )
	SET( HAPI_INCLUDE_DIR ${HAPI_INCLUDE_DIR} ${HAPI_SOURCE_DIR}/../OpenHapticsRenderer/include CACHE INTERNAL "Set to internal to propagate change" )
ENDIF(OPENHAPTICS_FOUND)

# Option to include all available external libraries, haptics devices etc.
# Otherwise only those required for release are included. This option should be
# disabled for release to avoid license issues.
IF( NOT DEFINED H3D_INCLUDE_GPL_EXTERNALS )
  SET( H3D_INCLUDE_GPL_EXTERNALS "ON" CACHE BOOL "Option to include all externals no matter their licenses. If a commercial license is used then this should be OFF." )
ENDIF( NOT DEFINED H3D_INCLUDE_GPL_EXTERNALS )

IF( H3D_INCLUDE_GPL_EXTERNALS )
  IF( NOT CHAI3D_INCLUDE_DIR )
    SET( CHAI3D_INCLUDE_DIR "NOTFOUND" CACHE PATH "" FORCE )
  ENDIF( NOT CHAI3D_INCLUDE_DIR )
ELSE( H3D_INCLUDE_GPL_EXTERNALS )
  SET( CHAI3D_INCLUDE_DIR "" CACHE PATH "" FORCE )
ENDIF( H3D_INCLUDE_GPL_EXTERNALS )

FIND_PACKAGE(Chai3D)
IF(CHAI3D_FOUND)
  SET(HAVE_CHAI3D 1)
  INCLUDE_DIRECTORIES( ${CHAI3D_INCLUDE_DIR}
                       ../Chai3DRenderer/include )
	SET( HAPI_INCLUDE_DIR ${HAPI_INCLUDE_DIR} ${HAPI_SOURCE_DIR}/../Chai3DRenderer/include CACHE INTERNAL "Set to internal to propagate change" )
  SET(Chai3D_requiredLibs ${Chai3D_requiredLibs} ${CHAI3D_LIBRARIES} )

  IF( NOT ${CHAI3D_VERSION} VERSION_LESS "2.0.0" )
    # No need to make a better comparasion than version 2.0.0 since the
    # way we get the version number might be unreliable since the version
    # number is not updated in every file for previous releases of chai3d.
    # Note that this might also break in the future if chai3d changes their
    # version handling, then FindChai3D.cmake needs to be updated.
    SET( CHAI3D_VERSION_2_0 "YES" )
    # Needed because glut is needed if chai3D is version 2.0.0 or above
    IF(WIN32)
      FIND_PACKAGE(GLUTWin)
    ELSE(WIN32)
      FIND_PACKAGE(GLUT)
    ENDIF(WIN32)

    IF(GLUT_FOUND)
      INCLUDE_DIRECTORIES( ${GLUT_INCLUDE_DIR} )
			IF( WIN32 )
				STRIP_AND_ADD_LIBRARY_DIRECTORIES( ${GLUT_LIBRARIES} )
			ELSE( WIN32 )
				SET(Chai3D_requiredLibs ${Chai3D_requiredLibs} ${GLUT_LIBRARIES} )
			ENDIF( WIN32 )
	  ELSE(GLUT_FOUND)
		  SET(HAVE_CHAI3D 0)
    ENDIF(GLUT_FOUND)
  ENDIF( NOT ${CHAI3D_VERSION} VERSION_LESS "2.0.0" )
ELSEIF( NOT WIN32 )
  INCLUDE_DIRECTORIES( ../Chai3DRenderer/include )
	SET( HAPI_INCLUDE_DIR ${HAPI_INCLUDE_DIR} ${HAPI_SOURCE_DIR}/../Chai3DRenderer/include CACHE INTERNAL "Set to internal to propagate change" )
ENDIF(CHAI3D_FOUND)

FIND_PACKAGE(DHD)
IF(DHD_FOUND)
  SET(HAVE_DHDAPI 1)
  INCLUDE_DIRECTORIES( ${DHD_INCLUDE_DIR} )
  SET(HAPI_optionalLibs ${HAPI_optionalLibs} ${DHD_LIBRARIES} )
ENDIF(DHD_FOUND)

FIND_PACKAGE(VirtuoseAPI)
IF(VIRTUOSE_FOUND)
  SET(HAVE_VIRTUOSEAPI 1)
  INCLUDE_DIRECTORIES( ${VIRTUOSE_INCLUDE_DIR} )
  SET(HAPI_optionalLibs ${HAPI_optionalLibs} ${VIRTUOSE_LIBRARIES} )
ENDIF(VIRTUOSE_FOUND)

FIND_PACKAGE(EntactAPI)
IF(ENTACTAPI_FOUND)
  SET(HAVE_ENTACTAPI 1)
  INCLUDE_DIRECTORIES( ${ENTACTAPI_INCLUDE_DIR} )
  SET(HAPI_optionalLibs ${HAPI_optionalLibs} ${ENTACTAPI_LIBRARIES} )
ENDIF(ENTACTAPI_FOUND)

FIND_PACKAGE(FalconAPI)
IF(FALCONAPI_FOUND)
  SET(HAVE_FALCONAPI 1)
  INCLUDE_DIRECTORIES( ${FALCONAPI_INCLUDE_DIR} )
  SET(HAPI_optionalLibs ${HAPI_optionalLibs} ${FALCONAPI_LIBRARIES} )
ENDIF(FALCONAPI_FOUND)

FIND_PACKAGE(NiFalconAPI)
IF(NIFALCONAPI_FOUND)
  SET(HAVE_NIFALCONAPI 1)
  INCLUDE_DIRECTORIES( ${NIFALCONAPI_INCLUDE_DIR} )
  SET(HAPI_optionalLibs ${HAPI_optionalLibs} ${NIFALCONAPI_LIBRARIES} )
ENDIF(NIFALCONAPI_FOUND)

FIND_PACKAGE(Haptik)
IF(HAPTIK_FOUND)
  SET(HAVE_HAPTIK_LIBRARY 1)
  INCLUDE_DIRECTORIES( ${HAPTIK_INCLUDE_DIR} )
  SET(HAPI_optionalLibs ${HAPI_optionalLibs} ${HAPTIK_LIBRARIES} )
ENDIF(HAPTIK_FOUND)

FIND_PACKAGE(SimballMedical)
IF(SIMBALLMEDICAL_FOUND)
  SET(HAVE_SIMBALLMEDICAL_API 1)
  INCLUDE_DIRECTORIES( ${SIMBALLMEDICAL_INCLUDE_DIR} )
  SET(HAPI_optionalLibs ${HAPI_optionalLibs} ${SIMBALLMEDICAL_LIBRARIES} )
ENDIF(SIMBALLMEDICAL_FOUND)

FIND_PACKAGE(MLHI)
IF(MLHI_FOUND)
  SET(HAVE_MLHI 1)
  INCLUDE_DIRECTORIES( ${MLHI_INCLUDE_DIR} )
  SET(HAPI_optionalLibs ${HAPI_optionalLibs} ${MLHI_LIBRARIES} )
ENDIF(MLHI_FOUND)

FIND_PACKAGE(OpenGL)
IF(OPENGL_FOUND)
  SET(HAVE_OPENGL 1)
  INCLUDE_DIRECTORIES( ${OPENGL_INCLUDE_DIR} )
  SET(HAPI_requiredLibs ${HAPI_requiredLibs} ${OPENGL_LIBRARIES} )
ENDIF(OPENGL_FOUND)

FIND_PACKAGE(H3Dfparser)
IF(fparser_FOUND)
  SET(HAVE_FPARSER 1)
  INCLUDE_DIRECTORIES( ${fparser_INCLUDE_DIRS} )
  SET(HAPI_requiredLibs ${HAPI_requiredLibs} ${fparser_LIBRARIES} )
ENDIF(fparser_FOUND)

# The current implementation only have support for Haptic Master on Windows.
# No use in looking for the libs/dlls on the system since the dlls for
# haptic master are run-time loaded.
IF(WIN32)
  SET( HAVE_HAPTIC_MASTER_API "YES" CACHE BOOL "Disable if you do not want haptic master support." )
ENDIF(WIN32)

# add all sources, they are added to variables called HAPI_SRCS,
# OH_SRCS and CHAI_SRCS defined in the included file. Header files are added
# to variables named HAPI_HEADERS, OH_HEADERS and CHAIR_HEADERS.
INCLUDE( ${HAPI_SOURCE_DIR}/HAPISourceFiles.txt )

IF( GENERATE_UNITY_BUILD )
# Generate a unity build, by creating the UnityBuild.cpp and only including the required 
# source files.

SET( HAPI_SRCS "${HAPI_SOURCE_DIR}/../src/UnityBuild.cpp" )

SET( UNITYBUILD_INCLUDES "" )

foreach(filename ${HAPI_SOURCE_FILES})
  SET( UNITYBUILD_INCLUDES "${UNITYBUILD_INCLUDES}\n #include \"${filename}\"" )
endforeach(filename)

CONFIGURE_FILE( ${HAPI_SOURCE_DIR}/UnityBuild.cmake ${HAPI_SOURCE_DIR}/../src/UnityBuild.cpp )

ENDIF( GENERATE_UNITY_BUILD )

# add the HAPI.rc resource file if Visual Studio
IF(MSVC)
  SET( HAPI_SRCS ${HAPI_SRCS} ${CMAKE_CURRENT_BINARY_DIR}/HAPI.rc )
ENDIF(MSVC)

IF(WIN32)

  # OpenHapticsRenderer and Chai3DRenderer are compiled as separate dlls 
  # on Windows 
  ADD_LIBRARY(HAPI SHARED ${HAPI_SRCS} ${HAPI_HEADERS} )
  IF(OPENHAPTICS_FOUND)
    IF(MSVC)
      SET( OH_SRCS ${OH_SRCS} ${CMAKE_CURRENT_BINARY_DIR}/OpenHapticsRenderer.rc )
    ENDIF(MSVC)
    ADD_LIBRARY(OpenHapticsRenderer SHARED ${OH_SRCS} ${OH_HEADERS} )
    IF( GENERATE_CPACK_PROJECT )
      SET( INSTALL_RUNTIME_AND_LIBRARIES_ONLY_DEPENDENCIES ${INSTALL_RUNTIME_AND_LIBRARIES_ONLY_DEPENDENCIES} OpenHapticsRenderer )
    ENDIF( GENERATE_CPACK_PROJECT )
  ENDIF(OPENHAPTICS_FOUND)
  
  IF(HAVE_CHAI3D)
    IF(MSVC)
      SET( CHAI_SRCS ${CHAI_SRCS} ${CMAKE_CURRENT_BINARY_DIR}/Chai3DRenderer.rc )
    ENDIF(MSVC)
    ADD_LIBRARY(Chai3DRenderer SHARED ${CHAI_SRCS} ${CHAI_HEADERS} )
    IF( GENERATE_CPACK_PROJECT )
      SET( INSTALL_RUNTIME_AND_LIBRARIES_ONLY_DEPENDENCIES ${INSTALL_RUNTIME_AND_LIBRARIES_ONLY_DEPENDENCIES} Chai3DRenderer )
    ENDIF( GENERATE_CPACK_PROJECT )
  ENDIF(HAVE_CHAI3D)

  IF( MSVC )
    # make sure that HAPI.rc, OpenHapticsRenderer.rc and Chai3DRenderer.rc contains the correct svn-version
    # and autogenerate the resource files.
    SET( HAPI_SVN_VERSION "0" )
    # Find SubWCRev.exe
    FIND_FILE( SubWCRev
               NAMES "SubWCRev.exe"
               DOC   "Set to SubWCRev.exe that comes with TortoiseSVN. Used to find svn revision number." )
    MARK_AS_ADVANCED( SubWCRev )

    SET( HAPI_is_working_copy 10 )
    IF( SubWCRev )
      EXECUTE_PROCESS( COMMAND ${SubWCRev} ${HAPI_SOURCE_DIR}/../
                       RESULT_VARIABLE HAPI_is_working_copy )
      IF( ${HAPI_is_working_copy} EQUAL 0 )
        SET( HAPI_SVN_VERSION "$WCREV$" )
      ENDIF( ${HAPI_is_working_copy} EQUAL 0 )
    ENDIF( SubWCRev )

    # autogenerate HAPI.rc depending on the version
    CONFIGURE_FILE( HAPI.rc.cmake ${CMAKE_CURRENT_BINARY_DIR}/HAPI.rc )
    
    ADD_CUSTOM_COMMAND( TARGET HAPI
                        PRE_BUILD
                        COMMAND ${HAPI_SOURCE_DIR}/UpdateResourceFile 
                        ARGS HAPI ${CMAKE_CURRENT_BINARY_DIR}/HAPI.rc ${HAPI_SOURCE_DIR}/HAPI.rc.cmake
                        ${HAPI_MAJOR_VERSION} ${HAPI_MINOR_VERSION}
                        ${HAPI_BUILD_VERSION} "${HAPI_SVN_VERSION}"
                        "HAPI_Output_Name"
                        "$(TargetFileName)" )

    IF( SubWCRev AND ${HAPI_is_working_copy} EQUAL 0 )
      # Update SVN revision in file.
      EXECUTE_PROCESS( COMMAND ${SubWCRev} ${HAPI_SOURCE_DIR}/../ ${CMAKE_CURRENT_BINARY_DIR}/HAPI.rc ${CMAKE_CURRENT_BINARY_DIR}/HAPI.rc )
      
      ADD_CUSTOM_COMMAND( TARGET HAPI 
                          PRE_BUILD 
                          COMMAND ${SubWCRev} 
                          ARGS ${HAPI_SOURCE_DIR}/../ ${CMAKE_CURRENT_BINARY_DIR}/HAPI.rc ${CMAKE_CURRENT_BINARY_DIR}/HAPI.rc )
    ENDIF( SubWCRev AND ${HAPI_is_working_copy} EQUAL 0 )

    IF(OPENHAPTICS_FOUND)
      # autogenerate HAPI.rc depending on the version
      CONFIGURE_FILE( OpenHapticsRenderer.rc.cmake ${CMAKE_CURRENT_BINARY_DIR}/OpenHapticsRenderer.rc )
      
      ADD_CUSTOM_COMMAND( TARGET OpenHapticsRenderer
                          PRE_BUILD
                          COMMAND ${HAPI_SOURCE_DIR}/UpdateResourceFile 
                          ARGS HAPI ${CMAKE_CURRENT_BINARY_DIR}/OpenHapticsRenderer.rc ${HAPI_SOURCE_DIR}/OpenHapticsRenderer.rc.cmake
                          ${HAPI_MAJOR_VERSION} ${HAPI_MINOR_VERSION}
                          ${HAPI_BUILD_VERSION} "${HAPI_SVN_VERSION}"
                          "OpenHapticsRenderer_Output_Name"
                          "$(TargetFileName)" )

      IF( SubWCRev AND ${HAPI_is_working_copy} EQUAL 0 )
        # Update SVN revision in file.
        EXECUTE_PROCESS( COMMAND ${SubWCRev} ${HAPI_SOURCE_DIR}/../ ${CMAKE_CURRENT_BINARY_DIR}/OpenHapticsRenderer.rc ${CMAKE_CURRENT_BINARY_DIR}/OpenHapticsRenderer.rc )
        
        ADD_CUSTOM_COMMAND( TARGET OpenHapticsRenderer 
                            PRE_BUILD 
                            COMMAND ${SubWCRev} 
                            ARGS ${HAPI_SOURCE_DIR}/../ ${CMAKE_CURRENT_BINARY_DIR}/OpenHapticsRenderer.rc ${CMAKE_CURRENT_BINARY_DIR}/OpenHapticsRenderer.rc )
      ENDIF( SubWCRev AND ${HAPI_is_working_copy} EQUAL 0 )
    ENDIF(OPENHAPTICS_FOUND)

    IF(HAVE_CHAI3D)
      # autogenerate Chai3DRenderer.rc depending on the version
      CONFIGURE_FILE( Chai3DRenderer.rc.cmake ${CMAKE_CURRENT_BINARY_DIR}/Chai3DRenderer.rc )
      
      ADD_CUSTOM_COMMAND( TARGET Chai3DRenderer
                          PRE_BUILD
                          COMMAND ${HAPI_SOURCE_DIR}/UpdateResourceFile 
                          ARGS HAPI ${CMAKE_CURRENT_BINARY_DIR}/Chai3DRenderer.rc ${HAPI_SOURCE_DIR}/Chai3DRenderer.rc.cmake
                          ${HAPI_MAJOR_VERSION} ${HAPI_MINOR_VERSION}
                          ${HAPI_BUILD_VERSION} "${HAPI_SVN_VERSION}"
                          "Chai3DRenderer_Output_Name"
                          "$(TargetFileName)" )

      IF( SubWCRev AND ${HAPI_is_working_copy} EQUAL 0 )
        # Update SVN revision in file.
        EXECUTE_PROCESS( COMMAND ${SubWCRev} ${HAPI_SOURCE_DIR}/../ ${CMAKE_CURRENT_BINARY_DIR}/Chai3DRenderer.rc ${CMAKE_CURRENT_BINARY_DIR}/Chai3DRenderer.rc )
        
        ADD_CUSTOM_COMMAND( TARGET Chai3DRenderer 
                            PRE_BUILD 
                            COMMAND ${SubWCRev} 
                            ARGS ${HAPI_SOURCE_DIR}/../ ${CMAKE_CURRENT_BINARY_DIR}/Chai3DRenderer.rc ${CMAKE_CURRENT_BINARY_DIR}/Chai3DRenderer.rc )
      ENDIF( SubWCRev AND ${HAPI_is_working_copy} EQUAL 0 )
    ENDIF(HAVE_CHAI3D)
  ENDIF(MSVC)
 
  # is this only for msvc or for all windows compilers?
  SET(HAPI_requiredLibs ${HAPI_requiredLibs} winmm.lib DelayImp.lib )
ELSE(WIN32)
  ADD_LIBRARY(HAPI SHARED ${HAPI_SRCS} ${OH_SRCS} ${CHAI_SRCS}
                          ${HAPI_HEADERS} ${OH_HEADERS} ${CHAI_HEADERS} )
ENDIF(WIN32)

# Needed to link correctly on MSVC70 and MSVC71 because the dependency is not enough
# to generate correct project files with this version of CMake.
IF( (MSVC70 OR MSVC71) AND ${CMAKE_MAJOR_VERSION}.${CMAKE_MINOR_VERSION} GREATER 2.7 )
  SET(HAPI_requiredLibs ${HAPI_requiredLibs} H3DUtil )
ENDIF( (MSVC70 OR MSVC71) AND ${CMAKE_MAJOR_VERSION}.${CMAKE_MINOR_VERSION} GREATER 2.7 )


# make sure symbols are exported.
SET( HAPI_COMPILE_FLAGS "-DHAPI_EXPORTS" )

# add the libraries needed for linking
TARGET_LINK_LIBRARIES( HAPI ${HAPI_requiredLibs} ${HAPI_optionalLibs} )

IF( COMMAND cmake_policy )
  IF( POLICY CMP0054 )
    cmake_policy(SET CMP0054 OLD)
  ENDIF( POLICY CMP0054 )
ENDIF( COMMAND cmake_policy )

foreach( H3DUTIL_INCLUDE_DIR_TMP ${H3DUTIL_INCLUDE_DIR} )
  IF( EXISTS ${H3DUTIL_INCLUDE_DIR_TMP}/H3DUtil/H3DUtil.h )
    FILE( STRINGS ${H3DUTIL_INCLUDE_DIR_TMP}/H3DUtil/H3DUtil.h list_of_defines REGEX "#define THREAD_LOCK_DEBUG"  )
    LIST( LENGTH list_of_defines list_of_defines_length )
    IF( list_of_defines_length )
      if ("${CMAKE_CXX_COMPILER_ID}" MATCHES "GNU")
        execute_process( COMMAND ${CMAKE_CXX_COMPILER} -dumpversion OUTPUT_VARIABLE GCC_VERSION)
        if (GCC_VERSION VERSION_GREATER 4.7 OR GCC_VERSION VERSION_EQUAL 4.7)
          add_definitions("-std=gnu++11")
        elseif(GCC_VERSION VERSION_GREATER 4.3 OR GCC_VERSION VERSION_EQUAL 4.3)
          add_definitions("-std=gnu++0x")
        else ()
          message(FATAL_ERROR "Enabling ENABLE_THREAD_LOCK_DEBUG requires C++11 support. This compiler lacks such support.")
        endif ()
      elseif (${MSVC_VERSION} LESS 1600 )
        message(FATAL_ERROR "Enabling ENABLE_THREAD_LOCK_DEBUG requires C++11 support. This compiler lacks such support.")
      endif ()
    ENDIF( list_of_defines_length )
  ENDIF( EXISTS ${H3DUTIL_INCLUDE_DIR_TMP}/H3DUtil/H3DUtil.h )
endforeach( H3DUTIL_INCLUDE_DIR_TMP )


# make the name of debug libraries end in _d.
SET_TARGET_PROPERTIES( HAPI PROPERTIES DEBUG_POSTFIX "_d" )

# set the version of the library
SET_TARGET_PROPERTIES( HAPI PROPERTIES VERSION ${HAPI_FULL_VERSION} )

IF(APPLE)
  SET_TARGET_PROPERTIES (HAPI
                         PROPERTIES
#                        BUILD_WITH_INSTALL_RPATH 1
                         INSTALL_NAME_DIR "${CMAKE_INSTALL_PREFIX}/lib")
ENDIF(APPLE)

IF(UNIX)
    SET_TARGET_PROPERTIES( HAPI PROPERTIES OUTPUT_NAME hapi )
ENDIF(UNIX)

SET( DEFAULT_BIN_INSTALL "bin" )
SET( DEFAULT_LIB_INSTALL "lib" )

IF( WIN32 )
  SET( DEFAULT_BIN_INSTALL "bin32" )
  SET( DEFAULT_LIB_INSTALL "lib32" )
  IF( CMAKE_SIZEOF_VOID_P EQUAL 8 )
    SET( DEFAULT_BIN_INSTALL "bin64" )
    SET( DEFAULT_LIB_INSTALL "lib64" )
  ENDIF( CMAKE_SIZEOF_VOID_P EQUAL 8 )

  SET( H3D_MSVC_VERSION "" )
  IF( MSVC )
    SET( H3D_MSVC_VERSION 6 )
    SET( TEMP_MSVC_VERSION 1299 )
    WHILE( ${MSVC_VERSION} GREATER ${TEMP_MSVC_VERSION} )
      MATH( EXPR H3D_MSVC_VERSION "${H3D_MSVC_VERSION} + 1" )
      MATH( EXPR TEMP_MSVC_VERSION "${TEMP_MSVC_VERSION} + 100" )
    ENDWHILE( ${MSVC_VERSION} GREATER ${TEMP_MSVC_VERSION} )
    SET( H3D_MSVC_VERSION "_vc${H3D_MSVC_VERSION}" )
    
    # change the name depending on compiler to be able to tell them apart
    # since they are not compatible with each other. 
    SET_TARGET_PROPERTIES( HAPI PROPERTIES OUTPUT_NAME ${PROJECT_NAME}${H3D_MSVC_VERSION} )
    
    # Treat wchar_t as built in type for all visual studio versions.
    # This is default for every version above 7 ( so far ) but we still set it for all.
    SET( HAPI_COMPILE_FLAGS "${HAPI_COMPILE_FLAGS} /Zc:wchar_t")
    
    IF( MSVC80 )
      # This might be useful for visual studio 2005 users that often recompile the api.
      IF( NOT DEFINED USE_VC8_MP_FLAG )
        SET( USE_VC8_MP_FLAG "NO" CACHE BOOL "In visual studio 8 the MP flag exists but is not documented. Maybe it is unsafe to use. If you want to use it then set this flag to yes." )
      ENDIF( NOT DEFINED USE_VC8_MP_FLAG )

      IF( USE_VC8_MP_FLAG )
        SET( HAPI_COMPILE_FLAGS "${HAPI_COMPILE_FLAGS} /MP" )
      ENDIF( USE_VC8_MP_FLAG )
    ENDIF( MSVC80 )

		# HAPI can not link incrementally on vc8 for some reason. Seems to be an issue for
		# vc9 also in some cases.
    SET( HAPI_LINK_FLAGS "${HAPI_LINK_FLAGS} /INCREMENTAL:NO" )
    
    IF( ${MSVC_VERSION} GREATER 1499 )
      # Build using several threads for visual studio versions 9 and above.
      SET( HAPI_COMPILE_FLAGS "${HAPI_COMPILE_FLAGS} /MP" )
    ENDIF( ${MSVC_VERSION} GREATER 1499 )
  ENDIF( MSVC )

  # Set properties for OpenHapticsRenderer if the project is created.
  IF(OPENHAPTICS_FOUND)
    TARGET_LINK_LIBRARIES( OpenHapticsRenderer ${OpenHaptics_requiredLibs} ${OpenHaptics_requiredLibs} HAPI )
    # change the name depending on compiler to be able to tell them apart
    # since they are not compatible with each other. 
    SET_TARGET_PROPERTIES( OpenHapticsRenderer PROPERTIES OUTPUT_NAME OpenHapticsRenderer${H3D_MSVC_VERSION} )
      
    # make sure symbols are exported.
    SET( OpenHapticsRenderer_COMPILE_FLAGS "-DOPENHAPTICSRENDERER_EXPORTS" )
    
    # Apparently this MSVC check is needed when built with Ming system.
    IF( MSVC )
      IF( ${MSVC_VERSION} GREATER 1499 )
        # Remove compiler warnings about deprecation for visual studio versions 9 and above.
        SET( OpenHapticsRenderer_COMPILE_FLAGS "${OpenHapticsRenderer_COMPILE_FLAGS} /MP -D_CRT_SECURE_NO_DEPRECATE" )
      ENDIF( ${MSVC_VERSION} GREATER 1499 )
    ENDIF( MSVC )
    
    # make the name of debug libraries end in _d.
    SET_TARGET_PROPERTIES( OpenHapticsRenderer PROPERTIES DEBUG_POSTFIX "_d" )
    # set the version of the library
    SET_TARGET_PROPERTIES( OpenHapticsRenderer PROPERTIES VERSION ${HAPI_FULL_VERSION} )

    # Set compile flags
    SET_TARGET_PROPERTIES( OpenHapticsRenderer 
                           PROPERTIES COMPILE_FLAGS
                           "${OpenHapticsRenderer_COMPILE_FLAGS}" )

    # Set link flags
    IF( MSVC )
      SET_TARGET_PROPERTIES( OpenHapticsRenderer HAPI
                             PROPERTIES
                             LINK_FLAGS "/DELAYLOAD:\"HD.dll\" /INCREMENTAL:NO" )

    # When using OpenHaptics HAPI also need to delayload HD.dll.
      SET( HAPI_LINK_FLAGS "${HAPI_LINK_FLAGS} /DELAYLOAD:\"HD.dll\"" )
    ENDIF( MSVC )
  ENDIF(OPENHAPTICS_FOUND)

  # Set properties for Chai3DRenderer if the project is created.
  IF(HAVE_CHAI3D)
    TARGET_LINK_LIBRARIES( Chai3DRenderer ${Chai3D_requiredLibs} ${Chai3D_requiredLibs} HAPI )
    
    # change the name depending on compiler to be able to tell them apart
    # since they are not compatible with each other. 
    SET_TARGET_PROPERTIES( Chai3DRenderer PROPERTIES OUTPUT_NAME Chai3DRenderer${H3D_MSVC_VERSION} )
    
    # make sure symbols are exported.
		SET( Chai3DRenderer_COMPILE_FLAGS "-DCHAI3DRENDERER_EXPORTS" )
		IF( HAVE_CHAI3D AND CHAI3D_VERSION_2_0 AND GLUT_FOUND AND PREFER_FREEGLUT_STATIC_LIBRARIES )
			SET( Chai3DRenderer_COMPILE_FLAGS "${Chai3DRenderer_COMPILE_FLAGS} -DFREEGLUT_STATIC" )
		ENDIF( HAVE_CHAI3D AND CHAI3D_VERSION_2_0 AND GLUT_FOUND AND PREFER_FREEGLUT_STATIC_LIBRARIES )
    SET_TARGET_PROPERTIES( Chai3DRenderer 
                           PROPERTIES COMPILE_FLAGS ${Chai3DRenderer_COMPILE_FLAGS} )
    # make the name of debug libraries end in _d.
    SET_TARGET_PROPERTIES( Chai3DRenderer PROPERTIES DEBUG_POSTFIX "_d" )
    # set the version of the library
    SET_TARGET_PROPERTIES( Chai3DRenderer PROPERTIES VERSION ${HAPI_FULL_VERSION} )

    IF( MSVC )
      # Set link flags
      # Chai3DRenderer can not use incremental linking in debug. We shut it off for
      # all visual studio versions because it does not really matter.
      SET_TARGET_PROPERTIES( Chai3DRenderer
                             PROPERTIES
                             LINK_FLAGS_DEBUG "/NODEFAULTLIB:msvcrt"
                             LINK_FLAGS  "/INCREMENTAL:NO" )
    ENDIF( MSVC )
  ENDIF(HAVE_CHAI3D)

  IF( MSVC )
    # Set link flags for HAPI
    SET_TARGET_PROPERTIES( HAPI
                           PROPERTIES
                           LINK_FLAGS "${HAPI_LINK_FLAGS}
    /DELAYLOAD:\"dhd.dll\" 
    /DELAYLOAD:\"dhd64.dll\" 
    /DELAYLOAD:\"hdl.dll\"
    /DELAYLOAD:\"Haptik.Library.dll\"
    /DELAYLOAD:\"SimballMedicalHID.dll\" 
    /DELAYLOAD:\"VirtuoseDLL.dll\"
    /DELAYLOAD:\"FTD2XX.dll\"
	  /DELAYLOAD:\"EntactAPI.dll\"" )
  ENDIF( MSVC )
ENDIF( WIN32 )

# set compile flags for HAPI project
SET_TARGET_PROPERTIES( HAPI PROPERTIES COMPILE_FLAGS "${HAPI_COMPILE_FLAGS}" )

# autogenerate HAPI.h depending on the libraries available.
IF( EXISTS ${HAPI_SOURCE_DIR}/../include/HAPI/HAPI.h )
  FILE( REMOVE ${HAPI_SOURCE_DIR}/../include/HAPI/HAPI.h )
ENDIF( EXISTS ${HAPI_SOURCE_DIR}/../include/HAPI/HAPI.h )
CONFIGURE_FILE( ${HAPI_SOURCE_DIR}/../include/HAPI/HAPI.cmake ${CMAKE_CURRENT_BINARY_DIR}/include/HAPI/HAPI.h )

IF( NOT GENERATE_CPACK_PROJECT )
  # Install header files on non-windows system (e.g. Unix).
  INSTALL( FILES ${CMAKE_CURRENT_BINARY_DIR}/include/HAPI/HAPI.h
           DESTINATION ${HAPI_SOURCE_DIR}/../include/HAPI )
ENDIF( NOT GENERATE_CPACK_PROJECT )

# Where to install HAPI
INSTALL( TARGETS HAPI 
         LIBRARY DESTINATION ${DEFAULT_LIB_INSTALL} COMPONENT HAPI_cpack_runtime
         RUNTIME DESTINATION ${DEFAULT_BIN_INSTALL} COMPONENT HAPI_cpack_runtime
         ARCHIVE DESTINATION ${DEFAULT_LIB_INSTALL} COMPONENT HAPI_cpack_libraries )

IF(WIN32)
  IF(OPENHAPTICS_FOUND)
    # Where to install OpenHapticsRenderer
    INSTALL( TARGETS OpenHapticsRenderer
             LIBRARY DESTINATION ${DEFAULT_LIB_INSTALL} COMPONENT HAPI_cpack_runtime
             RUNTIME DESTINATION ${DEFAULT_BIN_INSTALL} COMPONENT HAPI_cpack_runtime
             ARCHIVE DESTINATION ${DEFAULT_LIB_INSTALL} COMPONENT HAPI_cpack_libraries )
  ENDIF(OPENHAPTICS_FOUND)

  IF(HAVE_CHAI3D)
    # Where to install Chai3DRenderer
    INSTALL( TARGETS Chai3DRenderer 
             LIBRARY DESTINATION ${DEFAULT_LIB_INSTALL} COMPONENT HAPI_cpack_runtime
             RUNTIME DESTINATION ${DEFAULT_BIN_INSTALL} COMPONENT HAPI_cpack_runtime
             ARCHIVE DESTINATION ${DEFAULT_LIB_INSTALL} COMPONENT HAPI_cpack_libraries )
  ENDIF(HAVE_CHAI3D)
ELSEIF( NOT GENERATE_CPACK_PROJECT )
  # Install header files on non-windows system (e.g. Unix).
  INSTALL( FILES ${HAPI_HEADERS} ${OH_HEADERS} ${CHAI_HEADERS}
           DESTINATION include/HAPI )
ENDIF(WIN32)

IF( NOT DEFINED HAPI_EXAMPLE_PROJECTS )
  SET( HAPI_EXAMPLE_PROJECTS "NO" CACHE BOOL "If set to YES the example projects distributed with HAPI will be included in the build." )
ENDIF( NOT DEFINED HAPI_EXAMPLE_PROJECTS )

IF( HAPI_EXAMPLE_PROJECTS )
  ADD_SUBDIRECTORY( ${HAPI_SOURCE_DIR}/../examples/build
                    ${CMAKE_CURRENT_BINARY_DIR}/examples )
  IF( TARGET HAPIDemo )
    SET( INSTALL_RUNTIME_AND_LIBRARIES_ONLY_DEPENDENCIES ${INSTALL_RUNTIME_AND_LIBRARIES_ONLY_DEPENDENCIES} HAPIDemo )
  ENDIF( TARGET HAPIDemo )
  IF( TARGET SpringExample )
    SET( INSTALL_RUNTIME_AND_LIBRARIES_ONLY_DEPENDENCIES ${INSTALL_RUNTIME_AND_LIBRARIES_ONLY_DEPENDENCIES} SpringExample )
  ENDIF( TARGET SpringExample )
  IF( TARGET SurfaceExample )
    SET( INSTALL_RUNTIME_AND_LIBRARIES_ONLY_DEPENDENCIES ${INSTALL_RUNTIME_AND_LIBRARIES_ONLY_DEPENDENCIES} SurfaceExample )
  ENDIF( TARGET SurfaceExample )
  IF( TARGET FeedbackBufferCollector )
    SET( INSTALL_RUNTIME_AND_LIBRARIES_ONLY_DEPENDENCIES ${INSTALL_RUNTIME_AND_LIBRARIES_ONLY_DEPENDENCIES} FeedbackBufferCollector )
  ENDIF( TARGET FeedbackBufferCollector )
  IF( TARGET SimpleThreadPrint )
    SET( INSTALL_RUNTIME_AND_LIBRARIES_ONLY_DEPENDENCIES ${INSTALL_RUNTIME_AND_LIBRARIES_ONLY_DEPENDENCIES} SimpleThreadPrint )
  ENDIF( TARGET SimpleThreadPrint )
  IF( TARGET SimpleThreadPrintLock )
    SET( INSTALL_RUNTIME_AND_LIBRARIES_ONLY_DEPENDENCIES ${INSTALL_RUNTIME_AND_LIBRARIES_ONLY_DEPENDENCIES} SimpleThreadPrintLock )
  ENDIF( TARGET SimpleThreadPrintLock )
  IF( TARGET PeriodicThreadCallbacks )
    SET( INSTALL_RUNTIME_AND_LIBRARIES_ONLY_DEPENDENCIES ${INSTALL_RUNTIME_AND_LIBRARIES_ONLY_DEPENDENCIES} PeriodicThreadCallbacks )
  ENDIF( TARGET PeriodicThreadCallbacks )
  IF( TARGET DirectXExample )
    SET( INSTALL_RUNTIME_AND_LIBRARIES_ONLY_DEPENDENCIES ${INSTALL_RUNTIME_AND_LIBRARIES_ONLY_DEPENDENCIES} DirectXExample )
  ENDIF( TARGET DirectXExample )
ENDIF( HAPI_EXAMPLE_PROJECTS )

IF( H3DAPI_SOURCE_DIR )
  SET( INSTALL_RUNTIME_AND_LIBRARIES_ONLY_DEPENDENCIES ${INSTALL_RUNTIME_AND_LIBRARIES_ONLY_DEPENDENCIES} PARENT_SCOPE )
ELSE( H3DAPI_SOURCE_DIR )
  include( ${HAPI_SOURCE_DIR}/HAPICPack.cmake )
ENDIF( H3DAPI_SOURCE_DIR )
