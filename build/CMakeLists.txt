if( COMMAND cmake_policy )
  if( POLICY CMP0003 )
    cmake_policy(SET CMP0003 OLD)
  endif( POLICY CMP0003 )
endif( COMMAND cmake_policy )

if(WIN32)
  cmake_minimum_required(VERSION 2.6.0)
endif(WIN32)
# Make include files in paths like /usr/local/include have preference before
# files in OSX frameworks. 
set(CMAKE_FIND_FRAMEWORK "LAST")

project (HAPI)

# set the install directory on Windows
if( WIN32 AND CMAKE_INSTALL_PREFIX_INITIALIZED_TO_DEFAULT AND NOT H3D_CMAKE_INSTALL_PREFIX_ALREADY_SET )
  set( CMAKE_INSTALL_PREFIX ${HAPI_SOURCE_DIR}/../.. CACHE PATH "Install path prefix, prepended onto install directories." FORCE )
  set( H3D_CMAKE_INSTALL_PREFIX_ALREADY_SET TRUE )
endif( WIN32 AND CMAKE_INSTALL_PREFIX_INITIALIZED_TO_DEFAULT AND NOT H3D_CMAKE_INSTALL_PREFIX_ALREADY_SET )

# Add a cache variable GENERATE_UNITY_BUILD to have the choice of selecting
# a unity build project. Default is NO.
if( NOT DEFINED GENERATE_UNITY_BUILD )
  set( GENERATE_UNITY_BUILD "NO" CACHE BOOL "Decides if a the generated project files should build through a unity build instead of a normal build. A unity builds packs all .cpp files into a UnityBuild.cpp file and then only include this in the project. This greatly reduces build times." )
endif( NOT DEFINED GENERATE_UNITY_BUILD )

# Add a cache variable GENERATE_CPACK_PROJECT to have the choice of generating a project
# for packaging HAPI. Default is NO since most people will not use this.
if( NOT DEFINED GENERATE_CPACK_PROJECT )
  set( GENERATE_CPACK_PROJECT "NO" CACHE BOOL "Decides if a cpack project should be generated. The project in the first loaded CMakeLists will configure CPack." )
  mark_as_advanced(GENERATE_CPACK_PROJECT)
endif( NOT DEFINED GENERATE_CPACK_PROJECT )

# Add a cache variable ENABLE_SVN_REVISION to have the choice of using
# SubWCRev.exe to embed svn revision number in generated DLLs.
# Default is YES for Visual Studio and NMake generators, NO otherwise.
if( MSVC AND NOT DEFINED ENABLE_SVN_REVISION )
  set( ENABLE_SVN_REVISION_DEFAULT "NO" )
  if( CMAKE_GENERATOR MATCHES "Visual Studio|NMake" )
    set( ENABLE_SVN_REVISION_DEFAULT "YES" )
  endif( CMAKE_GENERATOR MATCHES "Visual Studio|NMake" )
  set( ENABLE_SVN_REVISION ${ENABLE_SVN_REVISION_DEFAULT} CACHE BOOL "Use SubWCRev.exe to embed svn revision number in generated DLLs." )
endif( MSVC AND NOT DEFINED ENABLE_SVN_REVISION )

# Add a cache variable PREFER_STATIC_LIBRARIES to have the choice of generating a project
# linking against static libraries if they exist. Default is NO since most people will not use this.
if( NOT DEFINED PREFER_STATIC_LIBRARIES )
  set( PREFER_STATIC_LIBRARIES "NO" CACHE BOOL "Decides if CMake should prefer static libraries to dynamic libraries when both exist." )
  mark_as_advanced(PREFER_STATIC_LIBRARIES)
endif( NOT DEFINED PREFER_STATIC_LIBRARIES )

if( PREFER_STATIC_LIBRARIES )
  set( CMAKE_FIND_LIBRARY_SUFFIXES .a;${CMAKE_FIND_LIBRARY_SUFFIXES} )  
endif( PREFER_STATIC_LIBRARIES )


# include H3DUtil in the build
if( ( WIN32 OR NOT H3D_USE_DEPENDENCIES_ONLY ) AND EXISTS "${HAPI_SOURCE_DIR}/../../H3DUtil/build" )
  message( STATUS "Including H3DUtil" )
  add_subdirectory( ${HAPI_SOURCE_DIR}/../../H3DUtil/build
                    ${CMAKE_CURRENT_BINARY_DIR}/H3DUtil )
  
  get_directory_property( H3DUTIL_INCLUDE_DIR_TMP DIRECTORY ${HAPI_SOURCE_DIR}/../../H3DUtil/build INCLUDE_DIRECTORIES )
  set( H3DUTIL_INCLUDE_DIR ${H3DUTIL_INCLUDE_DIR_TMP} CACHE INTERNAL "Set to internal so H3DUtil and/or HAPI can change it." )
endif( ( WIN32 OR NOT H3D_USE_DEPENDENCIES_ONLY ) AND EXISTS "${HAPI_SOURCE_DIR}/../../H3DUtil/build" )

# Add the directory to INCLUDE_DIRECTORIES before any other statement to make sure that this header file is found first.
# This assumes that we always use "#include <path/include_header.h>" in our code even if the file is actually found in
# the same directory as the file that contains the include statement.
include_directories( ${CMAKE_CURRENT_BINARY_DIR}/include )
set( HAPI_INCLUDE_DIR ${CMAKE_CURRENT_BINARY_DIR}/include ${HAPI_SOURCE_DIR}/../include CACHE INTERNAL "Set to internal to propagate change" )

# include header directory.
include_directories( ${HAPI_SOURCE_DIR}/../include )

set( HAPI_MAJOR_VERSION 1 )
set( HAPI_MINOR_VERSION 3 )
set( HAPI_BUILD_VERSION 1 )

# Set here already in case a special NSIS template needs to be used by cpack.
set(CMAKE_MODULE_PATH ${HAPI_SOURCE_DIR}/modules ${HAPI_SOURCE_DIR}/modules/sharedModules )
if( COMMAND cmake_policy )
  if( POLICY CMP0011 )
    cmake_policy( SET CMP0011 NEW )
  endif( POLICY CMP0011 )
endif( COMMAND cmake_policy )
include( StripAndAddLibraryDirectories )

set( HAPI_FULL_VERSION ${HAPI_MAJOR_VERSION}.${HAPI_MINOR_VERSION}.${HAPI_BUILD_VERSION} )

# add all optional libraries to this variable, only needed for HAPI project.
set( HAPI_optionalLibs )

# add all required libraries to this variable, one for each project
set( HAPI_requiredLibs)
set( Chai3D_requiredLibs)
set( OpenHaptics_requiredLibs)

if( TARGET H3DUtil )
  # Target was created by ADD_SUBDIRECTORY command.
  include_directories( ${H3DUTIL_INCLUDE_DIR} ) 
  set( HAPI_requiredLibs ${HAPI_requiredLibs} H3DUtil )
else( TARGET H3DUtil )
  #H3DUtil
  find_package(H3DUtil REQUIRED)
  if(H3DUTIL_FOUND)
    include_directories( ${H3DUTIL_INCLUDE_DIR} ) 
    set(HAPI_requiredLibs ${HAPI_requiredLibs} ${H3DUTIL_LIBRARIES} )
  endif(H3DUTIL_FOUND)
endif( TARGET H3DUtil )

include( H3DCommonFunctions )
set( HAPI_LINK_FLAGS "" )

find_package(OpenHaptics)
if(OpenHaptics_FOUND)
  set(HAVE_OPENHAPTICS 1)
  set( OPENHAPTICS_DLL_NAMES "HD" )
  addDelayLoadFlagsFromNames( OPENHAPTICS_DLL_NAMES HAPI_LINK_FLAGS )
  include_directories( ${OpenHaptics_INCLUDE_DIRS} 
                       ../OpenHapticsRenderer/include )
  set( HAPI_INCLUDE_DIR ${HAPI_INCLUDE_DIR} ${HAPI_SOURCE_DIR}/../OpenHapticsRenderer/include CACHE INTERNAL "Set to internal to propagate change" )
  set(HAPI_optionalLibs ${HAPI_optionalLibs} ${OpenHaptics_LIBRARIES} )
  set(OpenHaptics_requiredLibs ${OpenHaptics_requiredLibs} ${OpenHaptics_LIBRARIES} )
elseif( NOT WIN32 )
  include_directories( ../OpenHapticsRenderer/include )
  set( HAPI_INCLUDE_DIR ${HAPI_INCLUDE_DIR} ${HAPI_SOURCE_DIR}/../OpenHapticsRenderer/include CACHE INTERNAL "Set to internal to propagate change" )
endif(OpenHaptics_FOUND)

# Option to include all available external libraries, haptics devices etc.
# Otherwise only those required for release are included. This option should be
# disabled for release to avoid license issues.
if( NOT DEFINED H3D_INCLUDE_GPL_EXTERNALS )
  set( H3D_INCLUDE_GPL_EXTERNALS "ON" CACHE BOOL "Option to include all externals no matter their licenses. If a commercial license is used then this should be OFF." )
endif( NOT DEFINED H3D_INCLUDE_GPL_EXTERNALS )

if( H3D_INCLUDE_GPL_EXTERNALS )
  if( NOT CHAI3D_INCLUDE_DIR )
    set( CHAI3D_INCLUDE_DIR "NOTFOUND" CACHE PATH "" FORCE )
  endif( NOT CHAI3D_INCLUDE_DIR )
else( H3D_INCLUDE_GPL_EXTERNALS )
  set( CHAI3D_INCLUDE_DIR "" CACHE PATH "" FORCE )
endif( H3D_INCLUDE_GPL_EXTERNALS )

find_package(Chai3D)
if(CHAI3D_FOUND)
  set(HAVE_CHAI3D 1)
  include_directories( ${CHAI3D_INCLUDE_DIR}
                       ../Chai3DRenderer/include )
  set( HAPI_INCLUDE_DIR ${HAPI_INCLUDE_DIR} ${HAPI_SOURCE_DIR}/../Chai3DRenderer/include CACHE INTERNAL "Set to internal to propagate change" )
  set(Chai3D_requiredLibs ${Chai3D_requiredLibs} ${CHAI3D_LIBRARIES} )

  if( NOT ${CHAI3D_VERSION} VERSION_LESS "2.0.0" )
    # No need to make a better comparasion than version 2.0.0 since the
    # way we get the version number might be unreliable since the version
    # number is not updated in every file for previous releases of chai3d.
    # Note that this might also break in the future if chai3d changes their
    # version handling, then FindChai3D.cmake needs to be updated.
    set( CHAI3D_VERSION_2_0 "YES" )
    # Needed because glut is needed if chai3D is version 2.0.0 or above
    find_package(GLUT)

    if(GLUT_FOUND)
      include_directories( ${GLUT_INCLUDE_DIR} )
      if( WIN32 )
        STRIP_AND_ADD_LIBRARY_DIRECTORIES( ${GLUT_LIBRARIES} )
      else( WIN32 )
        set(Chai3D_requiredLibs ${Chai3D_requiredLibs} ${GLUT_LIBRARIES} )
      endif( WIN32 )
    else(GLUT_FOUND)
      set(HAVE_CHAI3D 0)
    endif(GLUT_FOUND)
  endif( NOT ${CHAI3D_VERSION} VERSION_LESS "2.0.0" )
elseif( NOT WIN32 )
  include_directories( ../Chai3DRenderer/include )
  set( HAPI_INCLUDE_DIR ${HAPI_INCLUDE_DIR} ${HAPI_SOURCE_DIR}/../Chai3DRenderer/include CACHE INTERNAL "Set to internal to propagate change" )
endif(CHAI3D_FOUND)

find_package(DHD)
if(DHD_FOUND)
  set(HAVE_DHDAPI 1)
  set( DHD_DLL_NAMES "dhd" )
  if( DHD_DRD_FOUND )
    set( DHD_DLL_NAMES "drd" )
    set(HAVE_DRDAPI 1)
  endif()
  if( WIN32 AND CMAKE_SIZEOF_VOID_P EQUAL 8 )
    set( DHD_DLL_NAMES "${DHD_DLL_NAMES}64" )
  endif()
  addDelayLoadFlagsFromNames( DHD_DLL_NAMES HAPI_LINK_FLAGS )
  include_directories( ${DHD_INCLUDE_DIRS} )
  set(HAPI_optionalLibs ${HAPI_optionalLibs} ${DHD_LIBRARIES} )
endif(DHD_FOUND)

find_package(VirtuoseAPI)
if(VirtuoseAPI_FOUND)
  set(HAVE_VIRTUOSEAPI 1)
  include_directories( ${VirtuoseAPI_INCLUDE_DIRS} )
  set(HAPI_optionalLibs ${HAPI_optionalLibs} ${VirtuoseAPI_LIBRARIES} )
  addDelayLoadFlags( VirtuoseAPI_LIBRARIES HAPI_LINK_FLAGS )
endif(VirtuoseAPI_FOUND)

find_package(EntactAPI)
if(ENTACTAPI_FOUND)
  set(HAVE_ENTACTAPI 1)
  include_directories( ${ENTACTAPI_INCLUDE_DIRS} )
  set(HAPI_optionalLibs ${HAPI_optionalLibs} ${ENTACTAPI_LIBRARIES} )
  addDelayLoadFlags( ENTACTAPI_LIBRARIES HAPI_LINK_FLAGS )
endif(ENTACTAPI_FOUND)

find_package(FalconAPI)
if(FalconAPI_FOUND)
  set(HAVE_FALCONAPI 1)
  include_directories( ${FalconAPI_INCLUDE_DIRS} )
  set(HAPI_optionalLibs ${HAPI_optionalLibs} ${FalconAPI_LIBRARIES} )
  addDelayLoadFlags( FalconAPI_LIBRARIES HAPI_LINK_FLAGS )
endif(FalconAPI_FOUND)

find_package(NiFalconAPI)
if(NiFalconAPI_FOUND)
  set(HAVE_NIFALCONAPI 1)
  include_directories( ${NiFalconAPI_INCLUDE_DIRS} )
  set(HAPI_optionalLibs ${HAPI_optionalLibs} ${NiFalconAPI_LIBRARIES} )
  set( NiFalcon_DLL_NAMES "FTD2XX" )
  addDelayLoadFlagsFromNames( NiFalcon_DLL_NAMES HAPI_LINK_FLAGS )
endif(NiFalconAPI_FOUND)

find_package(Haptik)
if(Haptik_FOUND)
  set(HAVE_Haptik_LIBRARY 1)
  include_directories( ${Haptik_INCLUDE_DIRS} )
  set(HAPI_optionalLibs ${HAPI_optionalLibs} ${Haptik_LIBRARIES} )
  set( HAPTIK_DLL_NAMES "Haptik.Library" )
  addDelayLoadFlagsFromNames( HAPTIK_DLL_NAMES HAPI_LINK_FLAGS )
endif(Haptik_FOUND)

find_package(SimballMedical)
if(SimballMedical_FOUND)
  set(HAVE_SIMBALLMEDICAL_API 1)
  include_directories( ${SimballMedical_INCLUDE_DIR} )
  set(HAPI_optionalLibs ${HAPI_optionalLibs} ${SimballMedical_LIBRARIES} )
  addDelayLoadFlags( SimballMedical_LIBRARIES HAPI_LINK_FLAGS )
endif(SimballMedical_FOUND)

find_package(MLHI)
if(MLHI_FOUND)
  set(HAVE_MLHI 1)
  include_directories( ${MLHI_INCLUDE_DIRS} )
  set(HAPI_optionalLibs ${HAPI_optionalLibs} ${MLHI_LIBRARIES} )
endif(MLHI_FOUND)

find_package(OpenGL)
if(OPENGL_FOUND)
  set(HAVE_OPENGL 1)
  include_directories( ${OPENGL_INCLUDE_DIR} )
  set(HAPI_requiredLibs ${HAPI_requiredLibs} ${OPENGL_LIBRARIES} )
endif(OPENGL_FOUND)

find_package(fparser)
if(fparser_FOUND)
  set(HAVE_FPARSER 1)
  include_directories( ${fparser_INCLUDE_DIRS} )
  set(HAPI_requiredLibs ${HAPI_requiredLibs} ${fparser_LIBRARIES} )
endif(fparser_FOUND)

# The current implementation only have support for Haptic Master on Windows.
# No use in looking for the libs/dlls on the system since the dlls for
# haptic master are run-time loaded.
if(WIN32)
  set( HAVE_HAPTIC_MASTER_API "YES" CACHE BOOL "Disable if you do not want haptic master support." )
endif(WIN32)

# add all sources, they are added to variables called HAPI_SRCS,
# OH_SRCS and CHAI_SRCS defined in the included file. Header files are added
# to variables named HAPI_HEADERS, OH_HEADERS and CHAIR_HEADERS.
include( ${HAPI_SOURCE_DIR}/HAPISourceFiles.txt )

if( GENERATE_UNITY_BUILD )
  if( NOT DEFINED UNITY_BUILD_${PROJECT_NAME} )
    set(  UNITY_BUILD_${PROJECT_NAME} "YES" CACHE BOOL "Decides if a the generated project files should build through a unity build instead of a normal build. A unity builds packs all .cpp files into a UnityBuild.cpp file and then only include this in the project. This greatly reduces build times." )
  endif()

  if( UNITY_BUILD_${PROJECT_NAME} )
    # Generate a unity build, by creating the UnityBuild.cpp and only including the required 
    # source files.

    set( UNITYBUILD_INCLUDES "" )

    foreach(filename ${HAPI_SOURCE_FILES})
      set( UNITYBUILD_INCLUDES "${UNITYBUILD_INCLUDES}\n #include \"${HAPI_SOURCE_DIR}/../src/${filename}\"" )
    endforeach(filename)

    # Using a cached variable with our string in it. Because cmake doesn't support multi-line strings we have to replace the newlines with a delimiter, so we arbitrarily use +=+.
    string(REPLACE "
" "+=+" UnitybuildIncludesConverted ${UNITYBUILD_INCLUDES}) # Convert the file we're going to write to use our delimiter instead of newlines
    if(NOT (UNITY_BUILD_CACHE_${PROJECT_NAME}) OR NOT (UnitybuildIncludesConverted STREQUAL UNITY_BUILD_CACHE_${PROJECT_NAME})) # If we don't have the cache variable or if its contents don't match our new string then we write the unmodified new UnityBuild file and store the one with the swapped out delimiters in the cache variable
      message(STATUS "Updating UnityBuild.cpp for " ${PROJECT_NAME})
      string(REPLACE "
" "+=+" unityBuildCacheNew ${UNITYBUILD_INCLUDES})
      set(UNITY_BUILD_CACHE_${PROJECT_NAME} ${unityBuildCacheNew} CACHE INTERNAL "Used for determining if UnityBuild.cpp should be updated or not.")
      file(WRITE UnityBuild.cpp ${UNITYBUILD_INCLUDES})
    else()
      message(STATUS "Unitybuild.cpp for ${PROJECT_NAME} already up to date")
    endif()
    set( HAPI_SRCS "UnityBuild.cpp" )

  endif()
endif( GENERATE_UNITY_BUILD )

set( PRECOMPILED_HEADERS_${PROJECT_NAME} "YES" CACHE BOOL "Decides if a the generated project files should use precompiled headers. This greatly reduces build times after first build." )

# set up precompiled headers  
if( MSVC AND PRECOMPILED_HEADERS_${PROJECT_NAME})
  list( APPEND HAPI_HEADERS ${HAPI_SOURCE_DIR}/../include/HAPI/StdAfx.h )
  list( APPEND HAPI_SRCS ${HAPI_SOURCE_DIR}/../src/StdAfx.cpp )
  if( ${MSVC_VERSION} LESS 1900)
    set_source_files_properties( ${HAPI_SRCS}
                                 PROPERTIES COMPILE_FLAGS "/Zm900 /FIHAPI/StdAfx.h /YuHAPI/StdAfx.h" )
    set_source_files_properties( ${HAPI_SOURCE_DIR}/../src/StdAfx.cpp 
                                 PROPERTIES COMPILE_FLAGS "/Zm900 /YcHAPI/StdAfx.h" )
  else( ${MSVC_VERSION} LESS 1900)
    set_source_files_properties( ${HAPI_SRCS}
                                 PROPERTIES COMPILE_FLAGS "/FIHAPI/StdAfx.h /YuHAPI/StdAfx.h" )
    set_source_files_properties( ${HAPI_SOURCE_DIR}/../src/StdAfx.cpp 
                                 PROPERTIES COMPILE_FLAGS "/YcHAPI/StdAfx.h" )
  endif( ${MSVC_VERSION} LESS 1900)                               
endif()

# add the HAPI.rc resource file if Visual Studio
if(MSVC)
  set( HAPI_SRCS ${HAPI_SRCS} ${CMAKE_CURRENT_BINARY_DIR}/HAPI.rc )
endif(MSVC)

if(WIN32)

  # OpenHapticsRenderer and Chai3DRenderer are compiled as separate dlls 
  # on Windows 
  add_library(HAPI SHARED ${HAPI_SRCS} ${HAPI_HEADERS} )
  if(OpenHaptics_FOUND)
    if(MSVC)
      set( OH_SRCS ${OH_SRCS} ${CMAKE_CURRENT_BINARY_DIR}/OpenHapticsRenderer.rc )
    endif(MSVC)
    add_library(OpenHapticsRenderer SHARED ${OH_SRCS} ${OH_HEADERS} )
    if( GENERATE_CPACK_PROJECT )
      set( INSTALL_RUNTIME_AND_LIBRARIES_ONLY_DEPENDENCIES ${INSTALL_RUNTIME_AND_LIBRARIES_ONLY_DEPENDENCIES} OpenHapticsRenderer )
    endif( GENERATE_CPACK_PROJECT )
  endif(OpenHaptics_FOUND)
  
  if(HAVE_CHAI3D)
    if(MSVC)
      set( CHAI_SRCS ${CHAI_SRCS} ${CMAKE_CURRENT_BINARY_DIR}/Chai3DRenderer.rc )
    endif(MSVC)
    add_library(Chai3DRenderer SHARED ${CHAI_SRCS} ${CHAI_HEADERS} )
    if( GENERATE_CPACK_PROJECT )
      set( INSTALL_RUNTIME_AND_LIBRARIES_ONLY_DEPENDENCIES ${INSTALL_RUNTIME_AND_LIBRARIES_ONLY_DEPENDENCIES} Chai3DRenderer )
    endif( GENERATE_CPACK_PROJECT )
  endif(HAVE_CHAI3D)

  if( MSVC )
    # make sure that HAPI.rc, OpenHapticsRenderer.rc and Chai3DRenderer.rc contains the correct svn-version
    # and autogenerate the resource files.
    set( HAPI_SVN_VERSION "0" )
    
    if( ENABLE_SVN_REVISION )
      # Find SubWCRev.exe
      find_file( SubWCRev
                 NAMES "SubWCRev.exe"
                 DOC   "Set to SubWCRev.exe that comes with TortoiseSVN. Used to find svn revision number." )
      mark_as_advanced( SubWCRev )
    endif( ENABLE_SVN_REVISION )

    set( HAPI_is_working_copy 10 )
    if( ENABLE_SVN_REVISION AND SubWCRev )
      execute_process( COMMAND ${SubWCRev} ${HAPI_SOURCE_DIR}/../
                       RESULT_VARIABLE HAPI_is_working_copy )
      if( ${HAPI_is_working_copy} EQUAL 0 )
        set( HAPI_SVN_VERSION "$WCREV$" )
      endif( ${HAPI_is_working_copy} EQUAL 0 )
    endif( ENABLE_SVN_REVISION AND SubWCRev )

    # autogenerate HAPI.rc depending on the version
    configure_file( HAPI.rc.cmake ${CMAKE_CURRENT_BINARY_DIR}/HAPI.rc )

    if( ENABLE_SVN_REVISION )
      add_custom_command( TARGET HAPI
                          PRE_BUILD
                          COMMAND ${HAPI_SOURCE_DIR}/UpdateResourceFile 
                          ARGS HAPI ${CMAKE_CURRENT_BINARY_DIR}/HAPI.rc ${HAPI_SOURCE_DIR}/HAPI.rc.cmake
                          ${HAPI_MAJOR_VERSION} ${HAPI_MINOR_VERSION}
                          ${HAPI_BUILD_VERSION} "${HAPI_SVN_VERSION}"
                          "HAPI_Output_Name"
                          "$(TargetFileName)" )
    endif( ENABLE_SVN_REVISION )

    if( ENABLE_SVN_REVISION AND SubWCRev AND ${HAPI_is_working_copy} EQUAL 0 )
      # Update SVN revision in file.
      execute_process( COMMAND ${SubWCRev} ${HAPI_SOURCE_DIR}/../ ${CMAKE_CURRENT_BINARY_DIR}/HAPI.rc ${CMAKE_CURRENT_BINARY_DIR}/HAPI.rc )
      
      add_custom_command( TARGET HAPI 
                          PRE_BUILD 
                          COMMAND ${SubWCRev} 
                          ARGS ${HAPI_SOURCE_DIR}/../ ${CMAKE_CURRENT_BINARY_DIR}/HAPI.rc ${CMAKE_CURRENT_BINARY_DIR}/HAPI.rc )
    endif( ENABLE_SVN_REVISION AND SubWCRev AND ${HAPI_is_working_copy} EQUAL 0 )

    if(OpenHaptics_FOUND)
      # autogenerate HAPI.rc depending on the version
      configure_file( OpenHapticsRenderer.rc.cmake ${CMAKE_CURRENT_BINARY_DIR}/OpenHapticsRenderer.rc )
      
      if( ENABLE_SVN_REVISION )
        add_custom_command( TARGET OpenHapticsRenderer
                            PRE_BUILD
                            COMMAND ${HAPI_SOURCE_DIR}/UpdateResourceFile 
                            ARGS HAPI ${CMAKE_CURRENT_BINARY_DIR}/OpenHapticsRenderer.rc ${HAPI_SOURCE_DIR}/OpenHapticsRenderer.rc.cmake
                            ${HAPI_MAJOR_VERSION} ${HAPI_MINOR_VERSION}
                            ${HAPI_BUILD_VERSION} "${HAPI_SVN_VERSION}"
                            "OpenHapticsRenderer_Output_Name"
                            "$(TargetFileName)" )
      endif( ENABLE_SVN_REVISION )

      if( ENABLE_SVN_REVISION AND SubWCRev AND ${HAPI_is_working_copy} EQUAL 0 )
        # Update SVN revision in file.
        execute_process( COMMAND ${SubWCRev} ${HAPI_SOURCE_DIR}/../ ${CMAKE_CURRENT_BINARY_DIR}/OpenHapticsRenderer.rc ${CMAKE_CURRENT_BINARY_DIR}/OpenHapticsRenderer.rc )
        
        add_custom_command( TARGET OpenHapticsRenderer 
                            PRE_BUILD 
                            COMMAND ${SubWCRev} 
                            ARGS ${HAPI_SOURCE_DIR}/../ ${CMAKE_CURRENT_BINARY_DIR}/OpenHapticsRenderer.rc ${CMAKE_CURRENT_BINARY_DIR}/OpenHapticsRenderer.rc )
      endif( ENABLE_SVN_REVISION AND SubWCRev AND ${HAPI_is_working_copy} EQUAL 0 )
    endif(OpenHaptics_FOUND)

    if(HAVE_CHAI3D)
      # autogenerate Chai3DRenderer.rc depending on the version
      configure_file( Chai3DRenderer.rc.cmake ${CMAKE_CURRENT_BINARY_DIR}/Chai3DRenderer.rc )
      
      if( ENABLE_SVN_REVISION )
        add_custom_command( TARGET Chai3DRenderer
                            PRE_BUILD
                            COMMAND ${HAPI_SOURCE_DIR}/UpdateResourceFile 
                            ARGS HAPI ${CMAKE_CURRENT_BINARY_DIR}/Chai3DRenderer.rc ${HAPI_SOURCE_DIR}/Chai3DRenderer.rc.cmake
                            ${HAPI_MAJOR_VERSION} ${HAPI_MINOR_VERSION}
                            ${HAPI_BUILD_VERSION} "${HAPI_SVN_VERSION}"
                            "Chai3DRenderer_Output_Name"
                            "$(TargetFileName)" )
      endif( ENABLE_SVN_REVISION )

      if( ENABLE_SVN_REVISION AND SubWCRev AND ${HAPI_is_working_copy} EQUAL 0 )
        # Update SVN revision in file.
        execute_process( COMMAND ${SubWCRev} ${HAPI_SOURCE_DIR}/../ ${CMAKE_CURRENT_BINARY_DIR}/Chai3DRenderer.rc ${CMAKE_CURRENT_BINARY_DIR}/Chai3DRenderer.rc )
        
        add_custom_command( TARGET Chai3DRenderer 
                            PRE_BUILD 
                            COMMAND ${SubWCRev} 
                            ARGS ${HAPI_SOURCE_DIR}/../ ${CMAKE_CURRENT_BINARY_DIR}/Chai3DRenderer.rc ${CMAKE_CURRENT_BINARY_DIR}/Chai3DRenderer.rc )
      endif( ENABLE_SVN_REVISION AND SubWCRev AND ${HAPI_is_working_copy} EQUAL 0 )
    endif(HAVE_CHAI3D)
  endif(MSVC)
 
  # is this only for msvc or for all windows compilers?
  set(HAPI_requiredLibs ${HAPI_requiredLibs} winmm.lib DelayImp.lib )
else(WIN32)
  add_library(HAPI SHARED ${HAPI_SRCS} ${OH_SRCS} ${CHAI_SRCS}
                          ${HAPI_HEADERS} ${OH_HEADERS} ${CHAI_HEADERS} )
endif(WIN32)

# Needed to link correctly on MSVC70 and MSVC71 because the dependency is not enough
# to generate correct project files with this version of CMake.
if( (MSVC70 OR MSVC71) AND ${CMAKE_MAJOR_VERSION}.${CMAKE_MINOR_VERSION} GREATER 2.7 )
  set(HAPI_requiredLibs ${HAPI_requiredLibs} H3DUtil )
endif( (MSVC70 OR MSVC71) AND ${CMAKE_MAJOR_VERSION}.${CMAKE_MINOR_VERSION} GREATER 2.7 )


# make sure symbols are exported.
set( HAPI_COMPILE_FLAGS "-DHAPI_EXPORTS" )

# add the libraries needed for linking
target_link_libraries( HAPI ${HAPI_requiredLibs} ${HAPI_optionalLibs} )

if( COMMAND cmake_policy )
  if( POLICY CMP0054 )
    cmake_policy(SET CMP0054 OLD)
  endif( POLICY CMP0054 )
endif( COMMAND cmake_policy )

foreach( H3DUTIL_INCLUDE_DIR_TMP ${H3DUTIL_INCLUDE_DIR} )
  if( EXISTS ${H3DUTIL_INCLUDE_DIR_TMP}/H3DUtil/H3DUtil.h )
    file( STRINGS ${H3DUTIL_INCLUDE_DIR_TMP}/H3DUtil/H3DUtil.h list_of_defines REGEX "#define THREAD_LOCK_DEBUG"  )
    list( LENGTH list_of_defines list_of_defines_length )
    if( list_of_defines_length )
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
    endif( list_of_defines_length )
  endif( EXISTS ${H3DUTIL_INCLUDE_DIR_TMP}/H3DUtil/H3DUtil.h )
endforeach( H3DUTIL_INCLUDE_DIR_TMP )


# make the name of debug libraries end in _d.
set_target_properties( HAPI PROPERTIES DEBUG_POSTFIX "_d" )

# set the version of the library
set_target_properties( HAPI PROPERTIES VERSION ${HAPI_FULL_VERSION} )

if(APPLE)
  set_target_properties (HAPI
                         PROPERTIES
#                        BUILD_WITH_INSTALL_RPATH 1
                         INSTALL_NAME_DIR "${CMAKE_INSTALL_PREFIX}/lib")
endif(APPLE)

if(UNIX)
    set_target_properties( HAPI PROPERTIES OUTPUT_NAME hapi )
endif(UNIX)

set( DEFAULT_BIN_INSTALL "bin" )
set( DEFAULT_LIB_INSTALL "lib" )

if( WIN32 )
  set( DEFAULT_BIN_INSTALL "bin32" )
  set( DEFAULT_LIB_INSTALL "lib32" )
  if( CMAKE_SIZEOF_VOID_P EQUAL 8 )
    set( DEFAULT_BIN_INSTALL "bin64" )
    set( DEFAULT_LIB_INSTALL "lib64" )
  endif( CMAKE_SIZEOF_VOID_P EQUAL 8 )

  if( MSVC )
    # change the name depending on compiler to be able to tell them apart
    # since they are not compatible with each other. 
    setH3DMSVCOutputName( HAPI "HAPI" )
    # Add common H3D compile flags for microsoft visual studio
    addCommonH3DMSVCCompileFlags( HAPI_COMPILE_FLAGS )
  endif( MSVC )

  # Set properties for OpenHapticsRenderer if the project is created.
  if(OpenHaptics_FOUND)
    target_link_libraries( OpenHapticsRenderer ${OpenHaptics_requiredLibs} ${OpenHaptics_requiredLibs} HAPI )
    # change the name depending on compiler to be able to tell them apart
    # since they are not compatible with each other. 
    setH3DMSVCOutputName( OpenHapticsRenderer "OpenHapticsRenderer" )
      
    # make sure symbols are exported.
    set( OpenHapticsRenderer_COMPILE_FLAGS "-DOPENHAPTICSRENDERER_EXPORTS" )
    
    # Apparently this MSVC check is needed when built with Ming system.
    if( MSVC )
      if( ${MSVC_VERSION} GREATER 1499 )
        # Remove compiler warnings about deprecation for visual studio versions 9 and above.
        set( OpenHapticsRenderer_COMPILE_FLAGS "${OpenHapticsRenderer_COMPILE_FLAGS} /MP -D_CRT_SECURE_NO_DEPRECATE" )
      endif( ${MSVC_VERSION} GREATER 1499 )
    endif( MSVC )
    
    # make the name of debug libraries end in _d.
    set_target_properties( OpenHapticsRenderer PROPERTIES DEBUG_POSTFIX "_d" )
    # set the version of the library
    set_target_properties( OpenHapticsRenderer PROPERTIES VERSION ${HAPI_FULL_VERSION} )

    # Set compile flags
    set_target_properties( OpenHapticsRenderer 
                           PROPERTIES COMPILE_FLAGS
                           "${OpenHapticsRenderer_COMPILE_FLAGS}" )

    # Set link flags
    if( MSVC )
      set_target_properties( OpenHapticsRenderer HAPI
                             PROPERTIES
                             LINK_FLAGS "/DELAYLOAD:\"HD.dll\" /INCREMENTAL:NO" )

      # When using OpenHaptics HAPI also need to delayload HD.dll.
      set( HAPI_LINK_FLAGS "${HAPI_LINK_FLAGS} /INCREMENTAL:NO" )
    endif( MSVC )
  endif(OpenHaptics_FOUND)

  # Set properties for Chai3DRenderer if the project is created.
  if(HAVE_CHAI3D)
    target_link_libraries( Chai3DRenderer ${Chai3D_requiredLibs} ${Chai3D_requiredLibs} HAPI )
    
    # change the name depending on compiler to be able to tell them apart
    # since they are not compatible with each other. 
    setH3DMSVCOutputName( Chai3DRenderer "Chai3DRenderer" )
    
    # make sure symbols are exported.
    set( Chai3DRenderer_COMPILE_FLAGS "-DCHAI3DRENDERER_EXPORTS" )
    if( HAVE_CHAI3D AND CHAI3D_VERSION_2_0 AND GLUT_FOUND AND PREFER_FREEGLUT_STATIC_LIBRARIES )
      set( Chai3DRenderer_COMPILE_FLAGS "${Chai3DRenderer_COMPILE_FLAGS} -DFREEGLUT_STATIC" )
    endif( HAVE_CHAI3D AND CHAI3D_VERSION_2_0 AND GLUT_FOUND AND PREFER_FREEGLUT_STATIC_LIBRARIES )
    set_target_properties( Chai3DRenderer 
                           PROPERTIES COMPILE_FLAGS ${Chai3DRenderer_COMPILE_FLAGS} )
    # make the name of debug libraries end in _d.
    set_target_properties( Chai3DRenderer PROPERTIES DEBUG_POSTFIX "_d" )
    # set the version of the library
    set_target_properties( Chai3DRenderer PROPERTIES VERSION ${HAPI_FULL_VERSION} )

    if( MSVC )
      # Set link flags
      # Chai3DRenderer can not use incremental linking in debug. We shut it off for
      # all visual studio versions because it does not really matter.
      set_target_properties( Chai3DRenderer
                             PROPERTIES
                             LINK_FLAGS_DEBUG "/NODEFAULTLIB:msvcrt"
                             LINK_FLAGS  "/INCREMENTAL:NO" )
    endif( MSVC )
  endif(HAVE_CHAI3D)

  if( MSVC )
    # Set link flags for HAPI
    # The backslashes are there because apparently endline characters will screw up the command line for anyone that wants to use Fastbuild
    set_target_properties( HAPI
                           PROPERTIES
                           LINK_FLAGS "${HAPI_LINK_FLAGS}" )
  endif( MSVC )
endif( WIN32 )

# set compile flags for HAPI project
set_target_properties( HAPI PROPERTIES COMPILE_FLAGS "${HAPI_COMPILE_FLAGS}" )

if(MSVC)
  set_target_properties( HAPI PROPERTIES FOLDER "H3D" )
endif(MSVC)

# autogenerate HAPI.h depending on the libraries available.
if( EXISTS ${HAPI_SOURCE_DIR}/../include/HAPI/HAPI.h )
  file( REMOVE ${HAPI_SOURCE_DIR}/../include/HAPI/HAPI.h )
endif( EXISTS ${HAPI_SOURCE_DIR}/../include/HAPI/HAPI.h )
configure_file( ${HAPI_SOURCE_DIR}/../include/HAPI/HAPI.cmake ${CMAKE_CURRENT_BINARY_DIR}/include/HAPI/HAPI.h )

if( NOT GENERATE_CPACK_PROJECT )
  # Install header files on non-windows system (e.g. Unix).
  install( FILES ${CMAKE_CURRENT_BINARY_DIR}/include/HAPI/HAPI.h
           DESTINATION ${HAPI_SOURCE_DIR}/../include/HAPI )
endif( NOT GENERATE_CPACK_PROJECT )

# Where to install HAPI
install( TARGETS HAPI 
         LIBRARY DESTINATION ${DEFAULT_LIB_INSTALL} COMPONENT HAPI_cpack_runtime
         RUNTIME DESTINATION ${DEFAULT_BIN_INSTALL} COMPONENT HAPI_cpack_runtime
         ARCHIVE DESTINATION ${DEFAULT_LIB_INSTALL} COMPONENT HAPI_cpack_libraries )

if(WIN32)
  if(OpenHaptics_FOUND)
    # Where to install OpenHapticsRenderer
    install( TARGETS OpenHapticsRenderer
             LIBRARY DESTINATION ${DEFAULT_LIB_INSTALL} COMPONENT HAPI_cpack_runtime
             RUNTIME DESTINATION ${DEFAULT_BIN_INSTALL} COMPONENT HAPI_cpack_runtime
             ARCHIVE DESTINATION ${DEFAULT_LIB_INSTALL} COMPONENT HAPI_cpack_libraries )
  endif(OpenHaptics_FOUND)

  if(HAVE_CHAI3D)
    # Where to install Chai3DRenderer
    install( TARGETS Chai3DRenderer 
             LIBRARY DESTINATION ${DEFAULT_LIB_INSTALL} COMPONENT HAPI_cpack_runtime
             RUNTIME DESTINATION ${DEFAULT_BIN_INSTALL} COMPONENT HAPI_cpack_runtime
             ARCHIVE DESTINATION ${DEFAULT_LIB_INSTALL} COMPONENT HAPI_cpack_libraries )
  endif(HAVE_CHAI3D)
elseif( NOT GENERATE_CPACK_PROJECT )
  # Install header files on non-windows system (e.g. Unix).
  install( FILES ${HAPI_HEADERS} ${OH_HEADERS} ${CHAI_HEADERS}
           DESTINATION include/HAPI )
endif(WIN32)

if( NOT DEFINED HAPI_EXAMPLE_PROJECTS )
  set( HAPI_EXAMPLE_PROJECTS "NO" CACHE BOOL "If set to YES the example projects distributed with HAPI will be included in the build." )
endif( NOT DEFINED HAPI_EXAMPLE_PROJECTS )

if( HAPI_EXAMPLE_PROJECTS )
  add_subdirectory( ${HAPI_SOURCE_DIR}/../examples/build
                    ${CMAKE_CURRENT_BINARY_DIR}/examples )
  if( TARGET HAPIDemo )
    set( INSTALL_RUNTIME_AND_LIBRARIES_ONLY_DEPENDENCIES ${INSTALL_RUNTIME_AND_LIBRARIES_ONLY_DEPENDENCIES} HAPIDemo )
  endif( TARGET HAPIDemo )
  if( TARGET SpringExample )
    set( INSTALL_RUNTIME_AND_LIBRARIES_ONLY_DEPENDENCIES ${INSTALL_RUNTIME_AND_LIBRARIES_ONLY_DEPENDENCIES} SpringExample )
  endif( TARGET SpringExample )
  if( TARGET SurfaceExample )
    set( INSTALL_RUNTIME_AND_LIBRARIES_ONLY_DEPENDENCIES ${INSTALL_RUNTIME_AND_LIBRARIES_ONLY_DEPENDENCIES} SurfaceExample )
  endif( TARGET SurfaceExample )
  if( TARGET FeedbackBufferCollector )
    set( INSTALL_RUNTIME_AND_LIBRARIES_ONLY_DEPENDENCIES ${INSTALL_RUNTIME_AND_LIBRARIES_ONLY_DEPENDENCIES} FeedbackBufferCollector )
  endif( TARGET FeedbackBufferCollector )
  if( TARGET SimpleThreadPrint )
    set( INSTALL_RUNTIME_AND_LIBRARIES_ONLY_DEPENDENCIES ${INSTALL_RUNTIME_AND_LIBRARIES_ONLY_DEPENDENCIES} SimpleThreadPrint )
  endif( TARGET SimpleThreadPrint )
  if( TARGET SimpleThreadPrintLock )
    set( INSTALL_RUNTIME_AND_LIBRARIES_ONLY_DEPENDENCIES ${INSTALL_RUNTIME_AND_LIBRARIES_ONLY_DEPENDENCIES} SimpleThreadPrintLock )
  endif( TARGET SimpleThreadPrintLock )
  if( TARGET PeriodicThreadCallbacks )
    set( INSTALL_RUNTIME_AND_LIBRARIES_ONLY_DEPENDENCIES ${INSTALL_RUNTIME_AND_LIBRARIES_ONLY_DEPENDENCIES} PeriodicThreadCallbacks )
  endif( TARGET PeriodicThreadCallbacks )
  if( TARGET DirectXExample )
    set( INSTALL_RUNTIME_AND_LIBRARIES_ONLY_DEPENDENCIES ${INSTALL_RUNTIME_AND_LIBRARIES_ONLY_DEPENDENCIES} DirectXExample )
  endif( TARGET DirectXExample )
endif( HAPI_EXAMPLE_PROJECTS )

if( H3DAPI_SOURCE_DIR )
  set( INSTALL_RUNTIME_AND_LIBRARIES_ONLY_DEPENDENCIES ${INSTALL_RUNTIME_AND_LIBRARIES_ONLY_DEPENDENCIES} PARENT_SCOPE )
else( H3DAPI_SOURCE_DIR )
  include( ${HAPI_SOURCE_DIR}/HAPICPack.cmake )
endif( H3DAPI_SOURCE_DIR )
