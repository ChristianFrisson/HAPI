cmake_minimum_required( VERSION 2.8.7 )
cmake_policy( VERSION 2.8.7 )

# Make include files in paths like /usr/local/include have preference before
# files in OSX frameworks. 
set( CMAKE_FIND_FRAMEWORK "LAST" )

project( HAPI )

# Set here already in case a special NSIS template needs to be used by cpack.
set( CMAKE_MODULE_PATH "${HAPI_SOURCE_DIR}/localModules" "${HAPI_SOURCE_DIR}/modules" )

include( H3DCommonFunctions )
handleCommonCacheVar( CMAKE_INSTALL_PREFIX ${HAPI_SOURCE_DIR}/../.. GENERATE_H3D_PACKAGE_PROJECT H3D_PREFER_STATIC_LIBRARIES )
# include H3DUtil in the build
if( ( WIN32 OR ( ${CMAKE_PROJECT_NAME} STREQUAL "HAPI" ) ) AND EXISTS "${HAPI_SOURCE_DIR}/../../H3DUtil/build" )
  message( STATUS "Including H3DUtil" )
  add_subdirectory( ${HAPI_SOURCE_DIR}/../../H3DUtil/build
                    ${CMAKE_CURRENT_BINARY_DIR}/H3DUtil )
endif()

# Add the directory to INCLUDE_DIRECTORIES before any other statement to make sure that this header file is found first.
# This assumes that we always use "#include <path/include_header.h>" in our code even if the file is actually found in
# the same directory as the file that contains the include statement.
include_directories( ${CMAKE_CURRENT_BINARY_DIR}/include ${HAPI_SOURCE_DIR}/../include )

set( HAPI_MAJOR_VERSION 1 )
set( HAPI_MINOR_VERSION 3 )
set( HAPI_BUILD_VERSION 1 )

# Set here already in case a special NSIS template needs to be used by cpack.
set( CMAKE_MODULE_PATH "${HAPI_SOURCE_DIR}/localModules" "${HAPI_SOURCE_DIR}/modules" )
include( StripAndAddLibraryDirectories )

set( HAPI_FULL_VERSION ${HAPI_MAJOR_VERSION}.${HAPI_MINOR_VERSION}.${HAPI_BUILD_VERSION} )

# add all optional libraries to this variable, only needed for HAPI project.
set( HAPI_optionalLibs )

# add all required libraries to this variable, one for each project
set( HAPI_requiredLibs )
set( chai3d_required_libs )
set( openhaptics_required_libs )

set( tmp_h3d_include_dirs )
findIncludeDirsAndLibrariesForH3DProjects ( PROJECT_NAMES H3DUtil
                                            INCLUDE_DIRS_OUTPUT_VAR tmp_h3d_include_dirs
                                            LIBRARIES_OUTPUT_VAR HAPI_requiredLibs
                                            REQUIRED_PROJECTS H3DUtil )
include_directories( ${tmp_h3d_include_dirs} ) 

include( H3DCommonFunctions )
set( HAPI_LINK_FLAGS "" )

find_package( OpenHaptics )
if( OpenHaptics_FOUND )
  set( HAVE_OPENHAPTICS 1 )
  set( OPENHAPTICS_DLL_NAMES "HD" )
  addDelayLoadFlagsFromNames( OPENHAPTICS_DLL_NAMES HAPI_LINK_FLAGS )
  include_directories( ${OpenHaptics_INCLUDE_DIRS} 
                       ${HAPI_SOURCE_DIR}/../OpenHapticsRenderer/include )
  set( HAPI_optionalLibs ${HAPI_optionalLibs} ${OpenHaptics_LIBRARIES} )
  set( openhaptics_required_libs ${openhaptics_required_libs} ${OpenHaptics_LIBRARIES} )
elseif( NOT WIN32 )
  include_directories( ${HAPI_SOURCE_DIR}/../OpenHapticsRenderer/include )
endif()

# Option to include all available external libraries, haptics devices etc.
# Otherwise only those required for release are included. This option should be
# disabled for release to avoid license issues.
if( NOT DEFINED H3D_INCLUDE_GPL_EXTERNALS )
  set( H3D_INCLUDE_GPL_EXTERNALS "ON" CACHE BOOL "Option to include all externals no matter their licenses. If a commercial license is used then this should be OFF." )
endif()

if( H3D_INCLUDE_GPL_EXTERNALS )
  if( NOT Chai3D_INCLUDE_DIR )
    set( Chai3D_INCLUDE_DIR "NOTFOUND" CACHE PATH "" FORCE )
  endif()
else()
  set( Chai3D_INCLUDE_DIR "" CACHE PATH "" FORCE )
endif()

find_package( Chai3D )
if( Chai3D_FOUND )
  set( HAVE_CHAI3D 1 )
  include_directories( ${Chai3D_INCLUDE_DIRS}
                       ${HAPI_SOURCE_DIR}/../Chai3DRenderer/include )
  set( chai3d_required_libs ${chai3d_required_libs} ${Chai3D_LIBRARIES} )

  if( NOT ${Chai3D_VERSION} VERSION_LESS "2.0.0" )
    # No need to make a better comparasion than version 2.0.0 since the
    # way we get the version number might be unreliable since the version
    # number is not updated in every file for previous releases of chai3d.
    # Note that this might also break in the future if chai3d changes their
    # version handling, then FindChai3D.cmake needs to be updated.
    set( CHAI3D_VERSION_2_0 "YES" )
    # Needed because glut is needed if chai3D is version 2.0.0 or above
    find_package( GLUT )

    if( GLUT_FOUND )
      include_directories( ${GLUT_INCLUDE_DIR} )
      if( WIN32 )
        stripAndAddLibraryDirectories( ${GLUT_LIBRARIES} )
      else()
        set( chai3d_required_libs ${chai3d_required_libs} ${GLUT_LIBRARIES} )
      endif()
    else()
      set( HAVE_CHAI3D 0 )
    endif()
  endif()
elseif( NOT WIN32 )
  include_directories( ${HAPI_SOURCE_DIR}/../Chai3DRenderer/include )
endif()

find_package( DHD )
if( DHD_FOUND )
  set( HAVE_DHDAPI 1 )
  set( DHD_DLL_NAMES "dhd" )
  if( DHD_drd_FOUND )
    set( DHD_DLL_NAMES "drd" )
    set( HAVE_DRDAPI 1 )
  endif()
  if( WIN32 AND CMAKE_SIZEOF_VOID_P EQUAL 8 )
    set( DHD_DLL_NAMES "${DHD_DLL_NAMES}64" )
  endif()
  addDelayLoadFlagsFromNames( DHD_DLL_NAMES HAPI_LINK_FLAGS )
  include_directories( ${DHD_INCLUDE_DIRS} )
  set( HAPI_optionalLibs ${HAPI_optionalLibs} ${DHD_LIBRARIES} )
endif()

find_package( VirtuoseAPI )
if( VirtuoseAPI_FOUND )
  set( HAVE_VIRTUOSEAPI 1 )
  include_directories( ${VirtuoseAPI_INCLUDE_DIRS} )
  set( HAPI_optionalLibs ${HAPI_optionalLibs} ${VirtuoseAPI_LIBRARIES} )
endif()

find_package( EntactAPI )
if( EntactAPI_FOUND )
  set( HAVE_ENTACTAPI 1 )
  include_directories( ${EntactAPI_INCLUDE_DIRS} )
  set( HAPI_optionalLibs ${HAPI_optionalLibs} ${EntactAPI_LIBRARIES} )
  addDelayLoadFlags( EntactAPI_LIBRARIES HAPI_LINK_FLAGS )
endif()

find_package( FalconAPI )
if( FalconAPI_FOUND )
  set( HAVE_FALCONAPI 1 )
  include_directories( ${FalconAPI_INCLUDE_DIRS} )
  set( HAPI_optionalLibs ${HAPI_optionalLibs} ${FalconAPI_LIBRARIES} )
  addDelayLoadFlags( FalconAPI_LIBRARIES HAPI_LINK_FLAGS )
endif()

find_package( NiFalconAPI )
if( NiFalconAPI_FOUND )
  set( HAVE_NIFALCONAPI 1 )
  include_directories( ${NiFalconAPI_INCLUDE_DIRS} )
  set( HAPI_optionalLibs ${HAPI_optionalLibs} ${NiFalconAPI_LIBRARIES} )
  set( NiFalcon_DLL_NAMES "FTD2XX" )
  addDelayLoadFlagsFromNames( NiFalcon_DLL_NAMES HAPI_LINK_FLAGS )
endif()

find_package( Haptik )
if( Haptik_FOUND )
  set( HAVE_Haptik_LIBRARY 1 )
  include_directories( ${Haptik_INCLUDE_DIRS} )
  set( HAPI_optionalLibs ${HAPI_optionalLibs} ${Haptik_LIBRARIES} )
  set( HAPTIK_DLL_NAMES "Haptik.Library" )
  addDelayLoadFlagsFromNames( HAPTIK_DLL_NAMES HAPI_LINK_FLAGS )
endif()

find_package( SimballMedical )
if( SimballMedical_FOUND )
  set( HAVE_SIMBALLMEDICAL_API 1 )
  include_directories( ${SimballMedical_INCLUDE_DIRS} )
  set( HAPI_optionalLibs ${HAPI_optionalLibs} ${SimballMedical_LIBRARIES} )
  addDelayLoadFlags( SimballMedical_LIBRARIES HAPI_LINK_FLAGS )
endif()

find_package( MLHI )
if( MLHI_FOUND )
  set( HAVE_MLHI 1 )
  include_directories( ${MLHI_INCLUDE_DIRS} )
  set( HAPI_optionalLibs ${HAPI_optionalLibs} ${MLHI_LIBRARIES} )
endif()

find_package( OpenGL )
if( OPENGL_FOUND )
  set( HAVE_OPENGL 1 )
  include_directories( ${OPENGL_INCLUDE_DIR} )
  set( HAPI_requiredLibs ${HAPI_requiredLibs} ${OPENGL_LIBRARIES} )
endif()

find_package( fparser )
if( fparser_FOUND )
  set( HAVE_FPARSER 1 )
  include_directories( ${fparser_INCLUDE_DIRS} )
  set( HAPI_requiredLibs ${HAPI_requiredLibs} ${fparser_LIBRARIES} )
endif()

# The current implementation only have support for Haptic Master on Windows.
# No use in looking for the libs/dlls on the system since the dlls for
# haptic master are run-time loaded.
if( WIN32 )
  handleRenamingVariablesBackwardCompatibility( NEW_VARIABLE_NAMES USE_HapticMasterDevice
                                                OLD_VARIABLE_NAMES HAVE_HAPTIC_MASTER_API
                                                DOC_STRINGS "Disable if you do not want haptic master support." )
  set( USE_HapticMasterDevice "YES" CACHE BOOL "Disable if you do not want haptic master support." )
endif()
set( HAVE_HAPTIC_MASTER_API ${USE_HapticMasterDevice} ) # In order to get HAPI.h to generate with correct settings.

# add all sources, they are added to variables called HAPI_SRCS,
# OH_SRCS and CHAI_SRCS defined in the included file. Header files are added
# to variables named HAPI_HEADERS, OH_HEADERS and CHAIR_HEADERS.
include( ${HAPI_SOURCE_DIR}/HAPISourceFiles.txt )
handleUnityBuild( PROJECT_NAME ${PROJECT_NAME}
                  SOURCE_FILES ${HAPI_SOURCE_FILES}
                  OUTPUT_VARIABLE HAPI_SRCS
                  SOURCE_PREFIX_PATH ${HAPI_SOURCE_DIR}/../src/ )
handlePrecompiledHeaders( PROJECT_NAME ${PROJECT_NAME}
                          HEADERS_VARIABLE HAPI_HEADERS
                          SRCS_VARIABLE HAPI_SRCS
                          STDAFX_HEADER_LOCATION ${HAPI_SOURCE_DIR}/../include/HAPI/
                          STDAFX_SOURCE_LOCATION ${HAPI_SOURCE_DIR}/../src/ )

# add the HAPI.rc resource file if Visual Studio
if( MSVC )
  set( HAPI_SRCS ${HAPI_SRCS} ${CMAKE_CURRENT_BINARY_DIR}/HAPI.rc )
endif()

populateProjectIncludeDirectoriesCacheVar( VARIABLE_NAME HAPI_INCLUDE_DIRS
                                           DEPRECATED_VARIABLE_NAMES HAPI_INCLUDE_DIR )

if( WIN32 )

  # OpenHapticsRenderer and Chai3DRenderer are compiled as separate dlls 
  # on Windows 
  add_library( HAPI SHARED ${HAPI_SRCS} ${HAPI_HEADERS} )
  if( OpenHaptics_FOUND )
    if( MSVC )
      set( OH_SRCS ${OH_SRCS} ${CMAKE_CURRENT_BINARY_DIR}/OpenHapticsRenderer.rc )
    endif()
    add_library( OpenHapticsRenderer SHARED ${OH_SRCS} ${OH_HEADERS} )
    if( GENERATE_H3D_PACKAGE_PROJECT )
      set( INSTALL_RUNTIME_AND_LIBRARIES_ONLY_DEPENDENCIES ${INSTALL_RUNTIME_AND_LIBRARIES_ONLY_DEPENDENCIES} OpenHapticsRenderer )
    endif()
  endif()
  
  if( HAVE_CHAI3D )
    if( MSVC )
      set( CHAI_SRCS ${CHAI_SRCS} ${CMAKE_CURRENT_BINARY_DIR}/Chai3DRenderer.rc )
    endif()
    add_library( Chai3DRenderer SHARED ${CHAI_SRCS} ${CHAI_HEADERS} )
    if( GENERATE_H3D_PACKAGE_PROJECT )
      set( INSTALL_RUNTIME_AND_LIBRARIES_ONLY_DEPENDENCIES ${INSTALL_RUNTIME_AND_LIBRARIES_ONLY_DEPENDENCIES} Chai3DRenderer )
    endif()
  endif()

  if( MSVC )
    # make sure that HAPI.rc, OpenHapticsRenderer.rc and Chai3DRenderer.rc contains the correct svn-version
    # and autogenerate the resource files.
    setupResourceFile( HAPI
                       VERSION_PREFIX HAPI
                       SVN_DIR_CANDIDATE ${HAPI_SOURCE_DIR}/../
                       RESOURCE_FILE_CMAKE_TEMPLATE ${HAPI_SOURCE_DIR}/HAPI.rc.cmake
                       RESOURCE_FILE_OUTPUT_LOCATION ${CMAKE_CURRENT_BINARY_DIR}/HAPI.rc
                       UPDATERESOURCEFILE_EXE ${HAPI_SOURCE_DIR}/UpdateResourceFile
                       UPDATERESOURCEFILE_EXE_EXTRA_ARGS "HAPI_Output_Name" "$(TargetFileName)" )

    if( OpenHaptics_FOUND )
      setupResourceFile( OpenHapticsRenderer
                   VERSION_PREFIX HAPI
                   SVN_DIR_CANDIDATE ${HAPI_SOURCE_DIR}/../
                   RESOURCE_FILE_CMAKE_TEMPLATE ${HAPI_SOURCE_DIR}/OpenHapticsRenderer.rc.cmake
                   RESOURCE_FILE_OUTPUT_LOCATION ${CMAKE_CURRENT_BINARY_DIR}/OpenHapticsRenderer.rc
                   UPDATERESOURCEFILE_EXE ${HAPI_SOURCE_DIR}/UpdateResourceFile
                   UPDATERESOURCEFILE_EXE_EXTRA_ARGS "OpenHapticsRenderer_Output_Name" "$(TargetFileName)" )
    endif()

    if( HAVE_CHAI3D )
      setupResourceFile( Chai3DRenderer
                   VERSION_PREFIX HAPI
                   SVN_DIR_CANDIDATE ${HAPI_SOURCE_DIR}/../
                   RESOURCE_FILE_CMAKE_TEMPLATE ${HAPI_SOURCE_DIR}/Chai3DRenderer.rc.cmake
                   RESOURCE_FILE_OUTPUT_LOCATION ${CMAKE_CURRENT_BINARY_DIR}/Chai3DRenderer.rc
                   UPDATERESOURCEFILE_EXE ${HAPI_SOURCE_DIR}/UpdateResourceFile
                   UPDATERESOURCEFILE_EXE_EXTRA_ARGS "Chai3DRenderer_Output_Name" "$(TargetFileName)" )
    endif()
  endif()
 
  # is this only for msvc or for all windows compilers?
  set( HAPI_requiredLibs ${HAPI_requiredLibs} winmm.lib DelayImp.lib )
else()
  add_library( HAPI SHARED ${HAPI_SRCS} ${OH_SRCS} ${CHAI_SRCS}
                          ${HAPI_HEADERS} ${OH_HEADERS} ${CHAI_HEADERS} )
endif()

# Needed to link correctly on MSVC70 and MSVC71 because the dependency is not enough
# to generate correct project files with this version of CMake.
if( ( MSVC70 OR MSVC71 ) AND ${CMAKE_MAJOR_VERSION}.${CMAKE_MINOR_VERSION} GREATER 2.7 )
  set( HAPI_requiredLibs ${HAPI_requiredLibs} H3DUtil )
endif()


# make sure symbols are exported.
set( HAPI_COMPILE_FLAGS "-DHAPI_EXPORTS" )

# add the libraries needed for linking
target_link_libraries( HAPI ${HAPI_requiredLibs} ${HAPI_optionalLibs} )

handleCommonCacheVar( USE_thread_lock_debug FALSE )

# make the name of debug libraries end in _d.
set_target_properties( HAPI PROPERTIES DEBUG_POSTFIX "_d" )

# set the version of the library
set_target_properties( HAPI PROPERTIES VERSION ${HAPI_FULL_VERSION} )

if( APPLE )
  set_target_properties( HAPI
                         PROPERTIES
#                        BUILD_WITH_INSTALL_RPATH 1
                         INSTALL_NAME_DIR "${CMAKE_INSTALL_PREFIX}/lib" )
endif()

if( UNIX )
    set_target_properties( HAPI PROPERTIES OUTPUT_NAME hapi )
endif()

getDefaultH3DOutputDirectoryName( default_bin_install default_lib_install )

if( WIN32 )
  if( MSVC )
    # change the name depending on compiler to be able to tell them apart
    # since they are not compatible with each other. 
    setH3DMSVCOutputName( HAPI "HAPI" )
    # Add common H3D compile flags for microsoft visual studio
    addCommonH3DMSVCCompileFlags( HAPI_COMPILE_FLAGS )
  endif()

  # Set properties for OpenHapticsRenderer if the project is created.
  if( OpenHaptics_FOUND )
    target_link_libraries( OpenHapticsRenderer ${openhaptics_required_libs} ${openhaptics_required_libs} HAPI )
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
      endif()
    endif()
    
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
                             LINK_FLAGS "/DELAYLOAD:\"${OPENHAPTICS_DLL_NAMES}.dll\" /INCREMENTAL:NO" )

      set( HAPI_LINK_FLAGS "${HAPI_LINK_FLAGS} /INCREMENTAL:NO" )
    endif()
  endif()

  # Set properties for Chai3DRenderer if the project is created.
  if( HAVE_CHAI3D )
    target_link_libraries( Chai3DRenderer ${chai3d_required_libs} ${chai3d_required_libs} HAPI )
    
    # change the name depending on compiler to be able to tell them apart
    # since they are not compatible with each other. 
    setH3DMSVCOutputName( Chai3DRenderer "Chai3DRenderer" )
    
    # make sure symbols are exported.
    set( Chai3DRenderer_COMPILE_FLAGS "-DCHAI3DRENDERER_EXPORTS" )
    if( HAVE_CHAI3D AND CHAI3D_VERSION_2_0 AND GLUT_FOUND AND PREFER_FREEGLUT_STATIC_LIBRARIES )
      set( Chai3DRenderer_COMPILE_FLAGS "${Chai3DRenderer_COMPILE_FLAGS} -DFREEGLUT_STATIC" )
    endif()
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
    endif()
  endif()

  if( MSVC )
    # Set link flags for HAPI
    # The backslashes are there because apparently endline characters will screw up the command line for anyone that wants to use Fastbuild
    set_target_properties( HAPI
                           PROPERTIES
                           LINK_FLAGS "${HAPI_LINK_FLAGS}" )
  endif()
endif()

# set compile flags for HAPI project
set_target_properties( HAPI PROPERTIES COMPILE_FLAGS "${HAPI_COMPILE_FLAGS}" )

if( MSVC )
  set_target_properties( HAPI PROPERTIES FOLDER "H3D" )
endif()

# autogenerate HAPI.h depending on the libraries available.
if( EXISTS ${HAPI_SOURCE_DIR}/../include/HAPI/HAPI.h )
  file( REMOVE ${HAPI_SOURCE_DIR}/../include/HAPI/HAPI.h )
endif()
configure_file( ${HAPI_SOURCE_DIR}/../include/HAPI/HAPI.cmake ${CMAKE_CURRENT_BINARY_DIR}/include/HAPI/HAPI.h )

if( NOT GENERATE_H3D_PACKAGE_PROJECT )
  # Install header files on non-windows system (e.g. Unix).
  install( FILES ${CMAKE_CURRENT_BINARY_DIR}/include/HAPI/HAPI.h
           DESTINATION ${HAPI_SOURCE_DIR}/../include/HAPI )
endif()

# Where to install HAPI
install( TARGETS HAPI 
         LIBRARY DESTINATION ${default_lib_install} COMPONENT HAPI_cpack_runtime
         RUNTIME DESTINATION ${default_bin_install} COMPONENT HAPI_cpack_runtime
         ARCHIVE DESTINATION ${default_lib_install} COMPONENT HAPI_cpack_libraries )

if( WIN32 )
  if( OpenHaptics_FOUND )
    # Where to install OpenHapticsRenderer
    install( TARGETS OpenHapticsRenderer
             LIBRARY DESTINATION ${default_lib_install} COMPONENT HAPI_cpack_runtime
             RUNTIME DESTINATION ${default_bin_install} COMPONENT HAPI_cpack_runtime
             ARCHIVE DESTINATION ${default_lib_install} COMPONENT HAPI_cpack_libraries )
  endif()

  if( HAVE_CHAI3D )
    # Where to install Chai3DRenderer
    install( TARGETS Chai3DRenderer 
             LIBRARY DESTINATION ${default_lib_install} COMPONENT HAPI_cpack_runtime
             RUNTIME DESTINATION ${default_bin_install} COMPONENT HAPI_cpack_runtime
             ARCHIVE DESTINATION ${default_lib_install} COMPONENT HAPI_cpack_libraries )
  endif()
elseif( NOT GENERATE_H3D_PACKAGE_PROJECT )
  # Install header files on non-windows system (e.g. Unix).
  install( FILES ${HAPI_HEADERS} ${OH_HEADERS} ${CHAI_HEADERS}
           DESTINATION include/HAPI )
endif()

handleRenamingVariablesBackwardCompatibility( NEW_VARIABLE_NAMES GENERATE_HAPI_example_PROJECTS
                                              OLD_VARIABLE_NAMES HAPI_EXAMPLE_PROJECTS
                                              DOC_STRINGS "If set to YES the example projects distributed with HAPI will be included in the build." )

if( NOT DEFINED GENERATE_HAPI_example_PROJECTS )
  set( GENERATE_HAPI_example_PROJECTS NO CACHE BOOL "If set to YES the example projects distributed with HAPI will be included in the build." )
endif()

if( GENERATE_HAPI_example_PROJECTS )
  add_subdirectory( ${HAPI_SOURCE_DIR}/../examples/build
                    ${CMAKE_CURRENT_BINARY_DIR}/examples )
  if( TARGET HAPIDemo )
    set( INSTALL_RUNTIME_AND_LIBRARIES_ONLY_DEPENDENCIES ${INSTALL_RUNTIME_AND_LIBRARIES_ONLY_DEPENDENCIES} HAPIDemo )
  endif()
  if( TARGET SpringExample )
    set( INSTALL_RUNTIME_AND_LIBRARIES_ONLY_DEPENDENCIES ${INSTALL_RUNTIME_AND_LIBRARIES_ONLY_DEPENDENCIES} SpringExample )
  endif()
  if( TARGET SurfaceExample )
    set( INSTALL_RUNTIME_AND_LIBRARIES_ONLY_DEPENDENCIES ${INSTALL_RUNTIME_AND_LIBRARIES_ONLY_DEPENDENCIES} SurfaceExample )
  endif()
  if( TARGET FeedbackBufferCollector )
    set( INSTALL_RUNTIME_AND_LIBRARIES_ONLY_DEPENDENCIES ${INSTALL_RUNTIME_AND_LIBRARIES_ONLY_DEPENDENCIES} FeedbackBufferCollector )
  endif()
  if( TARGET SimpleThreadPrint )
    set( INSTALL_RUNTIME_AND_LIBRARIES_ONLY_DEPENDENCIES ${INSTALL_RUNTIME_AND_LIBRARIES_ONLY_DEPENDENCIES} SimpleThreadPrint )
  endif()
  if( TARGET SimpleThreadPrintLock )
    set( INSTALL_RUNTIME_AND_LIBRARIES_ONLY_DEPENDENCIES ${INSTALL_RUNTIME_AND_LIBRARIES_ONLY_DEPENDENCIES} SimpleThreadPrintLock )
  endif()
  if( TARGET PeriodicThreadCallbacks )
    set( INSTALL_RUNTIME_AND_LIBRARIES_ONLY_DEPENDENCIES ${INSTALL_RUNTIME_AND_LIBRARIES_ONLY_DEPENDENCIES} PeriodicThreadCallbacks )
  endif()
  if( TARGET DirectXExample )
    set( INSTALL_RUNTIME_AND_LIBRARIES_ONLY_DEPENDENCIES ${INSTALL_RUNTIME_AND_LIBRARIES_ONLY_DEPENDENCIES} DirectXExample )
  endif()
endif()

if( NOT ${CMAKE_PROJECT_NAME} STREQUAL "HAPI" )
  set( INSTALL_RUNTIME_AND_LIBRARIES_ONLY_DEPENDENCIES ${INSTALL_RUNTIME_AND_LIBRARIES_ONLY_DEPENDENCIES} PARENT_SCOPE )
else()
  include( ${HAPI_SOURCE_DIR}/HAPICPack.cmake )
endif()
