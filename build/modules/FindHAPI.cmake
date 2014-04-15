# - Find HAPI
# Find the native HAPI headers and libraries.
# Output:
#  HAPI_INCLUDE_DIR -  where to find HAPI.h, etc.
#  HAPI_LIBRARIES    - List of libraries when using HAPI.
#  HAPI_FOUND        - True if HAPI found.
#
# Input
# HAPI_REQUIRED_RENDERERS - Set to a list of names of supported external renderers.
# Allowed values are "OpenHapticsRenderer" and "Chai3DRenderer". Only used
# if HAPI_DECIDES_RENDERER_SUPPORT is 1.
# HAPI_DECIDES_RENDERER_SUPPORT - If 1 HAPI decides which renderers are supported
# by checking HAPI.h (if found). If 0 the HAPI_REQUIRED_RENDERERS list is used.
# Note that external dependencies of the renderers are also required if a renderer is
# desired for support. For example, setting a dependency on OpenHapticsRenderer means
# that SensAbles OpenHaptics libraries must exist on the system for HAPI to be found
# properly. If a library do not use any of the external renderers then simply leave
# HAPI_DECIDES_RENDERER_SUPPORT at its default value (not existing or 0).
GET_FILENAME_COMPONENT(module_file_path ${CMAKE_CURRENT_LIST_FILE} PATH )

# Look for the header file.
FIND_PATH(HAPI_INCLUDE_DIR NAMES HAPI/HAPI.h 
                           PATHS $ENV{H3D_ROOT}/../HAPI/include
                                 ../../HAPI/include
                                 ${module_file_path}/../../../HAPI/include
                                 ${module_file_path}/../../include
                           DOC "Path in which the file HAPI/HAPI.h is located." )
MARK_AS_ADVANCED(HAPI_INCLUDE_DIR)

# Look for the library.
SET( HAPI_LIBRARY_SUFFIX "" )
IF( MSVC )
  SET( H3D_MSVC_VERSION 6 )
  SET( TEMP_MSVC_VERSION 1299 )
  WHILE( ${MSVC_VERSION} GREATER ${TEMP_MSVC_VERSION} )
    MATH( EXPR H3D_MSVC_VERSION "${H3D_MSVC_VERSION} + 1" )
    MATH( EXPR TEMP_MSVC_VERSION "${TEMP_MSVC_VERSION} + 100" )
  ENDWHILE( ${MSVC_VERSION} GREATER ${TEMP_MSVC_VERSION} )
  SET( HAPI_LIBRARY_SUFFIX "_vc${H3D_MSVC_VERSION}" )
  SET( HAPI_NAME "HAPI${HAPI_LIBRARY_SUFFIX}" )
ELSE(MSVC)
  SET( HAPI_NAME hapi )
ENDIF( MSVC )

SET( DEFAULT_LIB_INSTALL "lib" )
IF( WIN32 )
  SET( DEFAULT_LIB_INSTALL "lib32" )
  IF( CMAKE_SIZEOF_VOID_P EQUAL 8 )
    SET( DEFAULT_LIB_INSTALL "lib64" )
  ENDIF( CMAKE_SIZEOF_VOID_P EQUAL 8 )
ENDIF( WIN32 )

FIND_LIBRARY(HAPI_LIBRARY NAMES ${HAPI_NAME}
                          PATHS $ENV{H3D_ROOT}/../${DEFAULT_LIB_INSTALL}
                                ../../${DEFAULT_LIB_INSTALL}
                                ${module_file_path}/../../../${DEFAULT_LIB_INSTALL}
                          DOC "Path to ${HAPI_NAME} library." )

FIND_LIBRARY( HAPI_DEBUG_LIBRARY NAMES ${HAPI_NAME}_d
              PATHS $ENV{H3D_ROOT}/../${DEFAULT_LIB_INSTALL}
                    ../../${DEFAULT_LIB_INSTALL}
                    ${module_file_path}/../../../${DEFAULT_LIB_INSTALL}
                    DOC "Path to ${HAPI_NAME}_d library." )
MARK_AS_ADVANCED(HAPI_LIBRARY)
MARK_AS_ADVANCED(HAPI_DEBUG_LIBRARY)

SET( HAPI_REQUIRED_RENDERERS_FOUND 1 )
SET( HAPI_RENDERERS_INCLUDE_DIR "" )
SET( HAPI_RENDERERS_LIBRARIES "" )
SET( HAPI_OPENHAPTICS_SUPPORT 0 )
SET( HAPI_CHAI3D_SUPPORT 0 )

IF( HAPI_LIBRARY OR HAPI_DEBUG_LIBRARY  )
  SET( HAVE_HAPI_LIBRARY 1 )
ELSE( HAPI_LIBRARY OR HAPI_DEBUG_LIBRARY  )
  SET( HAVE_HAPI_LIBRARY 0 )
ENDIF( HAPI_LIBRARY OR HAPI_DEBUG_LIBRARY  )

IF( HAVE_HAPI_LIBRARY AND HAPI_INCLUDE_DIR )
  foreach( HAPI_INCLUDE_DIR_TMP ${HAPI_INCLUDE_DIR} )
    IF( EXISTS ${HAPI_INCLUDE_DIR_TMP}/HAPI/HAPI.h )
      SET( regex_to_find "#define HAVE_OPENHAPTICS" )
      FILE( STRINGS ${HAPI_INCLUDE_DIR_TMP}/HAPI/HAPI.h list_of_defines REGEX ${regex_to_find} )
      LIST( LENGTH list_of_defines list_of_defines_length )
      IF( list_of_defines_length )
        SET( HAPI_OPENHAPTICS_SUPPORT 1 )
      ENDIF( list_of_defines_length )
    
      SET( regex_to_find "#define HAVE_CHAI3D" )
      FILE( STRINGS ${HAPI_INCLUDE_DIR_TMP}/HAPI/HAPI.h list_of_defines REGEX ${regex_to_find} )
      LIST( LENGTH list_of_defines list_of_defines_length )
      IF( list_of_defines_length )
        SET( HAPI_CHAI3D_SUPPORT 1 )
      ENDIF( list_of_defines_length )
    ENDIF( EXISTS ${HAPI_INCLUDE_DIR_TMP}/HAPI/HAPI.h )
  endforeach( HAPI_INCLUDE_DIR_TMP ${HAPI_INCLUDE_DIR} )

    
  IF( HAPI_DECIDES_RENDERER_SUPPORT AND NOT HAPI_REQUIRED_RENDERERS )
    IF( HAPI_OPENHAPTICS_SUPPORT )
      SET( HAPI_REQUIRED_RENDERERS "OpenHapticsRenderer" )
    ENDIF( HAPI_OPENHAPTICS_SUPPORT )
    IF( HAPI_CHAI3D_SUPPORT )
      SET( HAPI_REQUIRED_RENDERERS ${HAPI_REQUIRED_RENDERERS} "Chai3DRenderer" )
    ENDIF( HAPI_CHAI3D_SUPPORT )
  ENDIF( HAPI_DECIDES_RENDERER_SUPPORT AND NOT HAPI_REQUIRED_RENDERERS )

  foreach( renderer_name ${HAPI_REQUIRED_RENDERERS} )
    IF( ( ${renderer_name} STREQUAL "OpenHapticsRenderer" ) AND NOT HAPI_OPENHAPTICS_SUPPORT )
      SET( HAPI_COMPILED_WITH_SUPPORT_MESSAGE "The found version of HAPI is not compiled with support for OpenHapticsRenderer" )
      IF(HAPI_FIND_REQUIRED)
        MESSAGE(FATAL_ERROR "${HAPI_COMPILED_WITH_SUPPORT_MESSAGE}")
      ELSEIF(NOT HAPI_FIND_QUIETLY)
        MESSAGE(STATUS "${HAPI_COMPILED_WITH_SUPPORT_MESSAGE}")
      ENDIF(HAPI_FIND_REQUIRED)
    ENDIF( ( ${renderer_name} STREQUAL "OpenHapticsRenderer" ) AND NOT HAPI_OPENHAPTICS_SUPPORT )
    
    
    IF( ( ${renderer_name} STREQUAL "Chai3DRenderer" ) AND NOT HAPI_CHAI3D_SUPPORT )
      SET( HAPI_COMPILED_WITH_SUPPORT_MESSAGE "The found version of HAPI is not compiled with support for Chai3DRenderer" )
      IF(HAPI_FIND_REQUIRED)
        MESSAGE(FATAL_ERROR "${HAPI_COMPILED_WITH_SUPPORT_MESSAGE}")
      ELSEIF(NOT HAPI_FIND_QUIETLY)
        MESSAGE(STATUS "${HAPI_COMPILED_WITH_SUPPORT_MESSAGE}")
      ENDIF(HAPI_FIND_REQUIRED)
    ENDIF( ( ${renderer_name} STREQUAL "Chai3DRenderer" ) AND NOT HAPI_CHAI3D_SUPPORT )
    
    FIND_PATH(HAPI_${renderer_name}_INCLUDE_DIR NAMES HAPI/${renderer_name}.h 
                               PATHS $ENV{H3D_ROOT}/../HAPI/${renderer_name}/include
                                     ../../HAPI/${renderer_name}/include
                                     ${module_file_path}/../../../HAPI/${renderer_name}/include
                                     ${HAPI_INCLUDE_DIR}/../${renderer_name}/include
                               DOC "Path in which the file HAPI/${renderer_name}.h is located." )
    MARK_AS_ADVANCED(HAPI_${renderer_name}_INCLUDE_DIR)

    IF( WIN32 )  
      FIND_LIBRARY(HAPI_${renderer_name}_LIBRARY NAMES ${renderer_name}${HAPI_LIBRARY_SUFFIX}
                            PATHS $ENV{H3D_ROOT}/../${DEFAULT_LIB_INSTALL}
                                  ../../${DEFAULT_LIB_INSTALL}
                                  ${module_file_path}/../../../${DEFAULT_LIB_INSTALL}
                            DOC "Path to ${renderer_name}${HAPI_LIBRARY_SUFFIX} library." )

     FIND_LIBRARY( HAPI_${renderer_name}_DEBUG_LIBRARY NAMES ${renderer_name}${HAPI_LIBRARY_SUFFIX}_d
                   PATHS $ENV{H3D_ROOT}/../${DEFAULT_LIB_INSTALL}
                          ../../${DEFAULT_LIB_INSTALL}
                          ${module_file_path}/../../../${DEFAULT_LIB_INSTALL}
                          DOC "Path to ${renderer_name}${HAPI_LIBRARY_SUFFIX}_d library." )
      MARK_AS_ADVANCED(HAPI_${renderer_name}_LIBRARY)
      MARK_AS_ADVANCED(HAPI_${renderer_name}_DEBUG_LIBRARY)
    ENDIF( WIN32 )
    
    IF( HAPI_${renderer_name}_INCLUDE_DIR AND ( HAPI_${renderer_name}_LIBRARY OR ${HAPI_${renderer_name}_DEBUG_LIBRARY} ) )  
      SET( HAPI_RENDERERS_INCLUDE_DIR ${HAPI_RENDERERS_INCLUDE_DIR} ${HAPI_${renderer_name}_INCLUDE_DIR} )
      IF( WIN32 )    
        IF(HAPI_${renderer_name}_LIBRARY)
          SET(HAPI_RENDERERS_LIBRARIES ${HAPI_RENDERERS_LIBRARIES} optimized ${HAPI_${renderer_name}_LIBRARY} )
        ELSE(HAPI_${renderer_name}_LIBRARY)
          SET(HAPI_RENDERERS_LIBRARIES ${HAPI_RENDERERS_LIBRARIES} optimized ${renderer_name}${HAPI_LIBRARY_SUFFIX} )
          MESSAGE( STATUS "HAPI ${renderer_name} release libraries not found. Release build might not work." )
        ENDIF(HAPI_${renderer_name}_LIBRARY)
          
        IF(HAPI_${renderer_name}_DEBUG_LIBRARY)
          SET(HAPI_RENDERERS_LIBRARIES ${HAPI_RENDERERS_LIBRARIES} debug ${HAPI_${renderer_name}_DEBUG_LIBRARY} )
        ELSE(HAPI_${renderer_name}_DEBUG_LIBRARY)
          SET(HAPI_RENDERERS_LIBRARIES ${HAPI_RENDERERS_LIBRARIES} debug ${renderer_name}${HAPI_LIBRARY_SUFFIX}_d )
          MESSAGE( STATUS "HAPI ${renderer_name} debug libraries not found. Debug build might not work." )
        ENDIF(HAPI_${renderer_name}_DEBUG_LIBRARY)
      ENDIF( WIN32 )
      
      IF( ( ${renderer_name} STREQUAL "OpenHapticsRenderer" ) )
        # OpenHapticsRenderer library is found. Check for OpenHaptics on the system. It must exist for the library
        # using HAPI since it is assumed that the library using HAPI will include OpenHapticsRenderer.h.
        FIND_PACKAGE( OpenHaptics REQUIRED )
        IF(OPENHAPTICS_FOUND)
          SET( HAPI_RENDERERS_INCLUDE_DIR ${HAPI_RENDERERS_INCLUDE_DIR} ${OPENHAPTICS_INCLUDE_DIR} )
          SET( HAPI_RENDERERS_LIBRARIES ${HAPI_RENDERERS_LIBRARIES} ${OPENHAPTICS_LIBRARIES} )
        ENDIF(OPENHAPTICS_FOUND)
      ENDIF( ( ${renderer_name} STREQUAL "OpenHapticsRenderer" ) )
      
      IF( ( ${renderer_name} STREQUAL "Chai3DRenderer" ) )
        # Chai3DRenderer library is found. Check for chai3d on the system. It must exist for the library
        # using HAPI since it is assumed that the library using HAPI will include Chai3DRenderer.h.
        FIND_PACKAGE( Chai3D REQUIRED )
        IF(CHAI3D_FOUND)
          SET( HAPI_RENDERERS_INCLUDE_DIR ${HAPI_RENDERERS_INCLUDE_DIR} ${CHAI3D_INCLUDE_DIR} )
          SET( HAPI_RENDERERS_LIBRARIES ${HAPI_RENDERERS_LIBRARIES} ${CHAI3D_LIBRARIES} )
        ENDIF(CHAI3D_FOUND)
      ENDIF( ( ${renderer_name} STREQUAL "Chai3DRenderer" ) )
    ELSE( HAPI_${renderer_name}_INCLUDE_DIR AND ( HAPI_${renderer_name}_LIBRARY OR ${HAPI_${renderer_name}_DEBUG_LIBRARY} ) )
      SET( HAPI_REQUIRED_RENDERERS_FOUND 0 )
    ENDIF( HAPI_${renderer_name}_INCLUDE_DIR AND ( HAPI_${renderer_name}_LIBRARY OR ${HAPI_${renderer_name}_DEBUG_LIBRARY} ) )
  endforeach( renderer_name ${HAPI_REQUIRED_RENDERERS} )
ENDIF( HAVE_HAPI_LIBRARY AND HAPI_INCLUDE_DIR )

# Copy the results to the output variables.
IF(HAPI_INCLUDE_DIR AND HAVE_HAPI_LIBRARY AND HAPI_REQUIRED_RENDERERS_FOUND)
  SET(HAPI_FOUND 1)
  IF(HAPI_LIBRARY)
    SET(HAPI_LIBRARIES ${HAPI_LIBRARIES} optimized ${HAPI_LIBRARY} )
  ELSE(HAPI_LIBRARY)
    SET(HAPI_LIBRARIES ${HAPI_LIBRARIES} optimized ${HAPI_NAME} )
    MESSAGE( STATUS "HAPI release libraries not found. Release build might not work." )
  ENDIF(HAPI_LIBRARY)

  IF(HAPI_DEBUG_LIBRARY)
    SET(HAPI_LIBRARIES ${HAPI_LIBRARIES} debug ${HAPI_DEBUG_LIBRARY} )
  ELSE(HAPI_DEBUG_LIBRARY)
    SET(HAPI_LIBRARIES ${HAPI_LIBRARIES} debug ${HAPI_NAME}_d )
    MESSAGE( STATUS "HAPI debug libraries not found. Debug build might not work." )
  ENDIF(HAPI_DEBUG_LIBRARY)
  
  SET(HAPI_INCLUDE_DIR ${HAPI_INCLUDE_DIR} ${HAPI_RENDERERS_INCLUDE_DIR} )
  SET(HAPI_LIBRARIES ${HAPI_LIBRARIES} ${HAPI_RENDERERS_LIBRARIES} )
ELSE(HAPI_INCLUDE_DIR AND HAVE_HAPI_LIBRARY AND HAPI_REQUIRED_RENDERERS_FOUND)
  SET(HAPI_FOUND 0)
  SET(HAPI_LIBRARIES)
  SET(HAPI_INCLUDE_DIR)
ENDIF(HAPI_INCLUDE_DIR AND HAVE_HAPI_LIBRARY AND HAPI_REQUIRED_RENDERERS_FOUND)

# Report the results.
IF(NOT HAPI_FOUND)
  SET(HAPI_DIR_MESSAGE
    "HAPI was not found. Make sure HAPI_LIBRARY ( and/or HAPI_DEBUG_LIBRARY ) and HAPI_INCLUDE_DIR" )
  foreach( renderer_name ${HAPI_REQUIRED_RENDERERS} )
    SET(HAPI_DIR_MESSAGE "${HAPI_DIR_MESSAGE} and HAPI_${renderer_name}_INCLUDE_DIR")
    IF( WIN32 )
      SET(HAPI_DIR_MESSAGE "${HAPI_DIR_MESSAGE} and HAPI_${renderer_name}_LIBRARY ( and/or HAPI_${renderer_name}_DEBUG_LIBRARY )")
    ENDIF( WIN32 )
  endforeach( renderer_name ${HAPI_REQUIRED_RENDERERS} )
  SET(HAPI_DIR_MESSAGE "${HAPI_DIR_MESSAGE} are set.")
  IF(HAPI_FIND_REQUIRED)
    MESSAGE(FATAL_ERROR "${HAPI_DIR_MESSAGE}")
  ELSEIF(NOT HAPI_FIND_QUIETLY)
    MESSAGE(STATUS "${HAPI_DIR_MESSAGE}")
  ENDIF(HAPI_FIND_REQUIRED)
ENDIF(NOT HAPI_FOUND)
