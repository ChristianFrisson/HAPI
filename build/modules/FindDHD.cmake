# - Find DHD
# Find the native DHD headers and libraries.
#
#  DHD_INCLUDE_DIR -  where to find DHD headers
#  DHD_LIBRARIES    - List of libraries when using DHD.
#  DHD_FOUND        - True if DHD found.

GET_FILENAME_COMPONENT(module_file_path ${CMAKE_CURRENT_LIST_FILE} PATH )

IF( CMAKE_CL_64 )
  SET( LIB "lib64" )
ELSE( CMAKE_CL_64 )
  SET( LIB "lib32" )
ENDIF( CMAKE_CL_64 )

# Look for the header file.
FIND_PATH(DHD_INCLUDE_DIR NAMES dhdc.h 
                          PATHS $ENV{H3D_EXTERNAL_ROOT}/include
                                $ENV{H3D_EXTERNAL_ROOT}/include/DHD-API
                                $ENV{H3D_ROOT}/../External/include
                                $ENV{H3D_ROOT}/../External/include/DHD-API
                                ../../External/include
                                ../../External/include/DHD-API 
                                ${module_file_path}/../../../External/include
                                ${module_file_path}/../../../External/include/DHD-API
                          DOC "Path in which the file dhdc.h is located." )
MARK_AS_ADVANCED(DHD_INCLUDE_DIR)


# Look for the library.
IF(WIN32)
  FIND_LIBRARY(DHD_LIBRARY NAMES dhdms dhdms64
                           PATHS $ENV{H3D_EXTERNAL_ROOT}/${LIB}
                                 $ENV{H3D_ROOT}/../External/${LIB}
                                 ../../External/${LIB}
                                 ${module_file_path}/../../../External/${LIB}
                           DOC "Path to dhdms library." )
ELSE(WIN32)
  FIND_LIBRARY(DHD_LIBRARY NAMES dhd
                           PATHS $ENV{H3D_EXTERNAL_ROOT}/${LIB}
                                 $ENV{H3D_ROOT}/../External/${LIB}
                                 ../../External/${LIB}
                                 ${module_file_path}/../../../External/${LIB}
                           DOC "Path to dhd library." )

  IF(UNIX)
    FIND_LIBRARY( DHD_USB_LIBRARY NAMES usb
                  DOC "Path to usb library." )
    FIND_LIBRARY( DHD_PCISCAN_LIBRARY NAMES pciscan
                  DOC "Path to pciscan library." )
    MARK_AS_ADVANCED(DHD_USB_LIBRARY)
    MARK_AS_ADVANCED(DHD_PCISCAN_LIBRARY)
  ENDIF(UNIX)
ENDIF(WIN32)
MARK_AS_ADVANCED(DHD_LIBRARY)


# Copy the results to the output variables.
IF(DHD_INCLUDE_DIR AND DHD_LIBRARY)
  SET(DHD_FOUND 1)
  SET(DHD_LIBRARIES ${DHD_LIBRARY})
  SET(DHD_INCLUDE_DIR ${DHD_INCLUDE_DIR})
  IF(UNIX)
    IF(DHD_USB_LIBRARY AND DHD_PCISCAN_LIBRARY)
      SET(DHD_LIBRARIES ${DHD_LIBRARIES} ${DHD_USB_LIBRARY} ${DHD_PCISCAN_LIBRARY})
    ELSE(DHD_USB_LIBRARY AND DHD_PCISCAN_LIBRARY)
      SET(DHD_FOUND 0)
      SET(DHD_LIBRARIES)
      SET(DHD_INCLUDE_DIR)
    ENDIF(DHD_USB_LIBRARY AND DHD_PCISCAN_LIBRARY)
  ENDIF(UNIX)
ELSE(DHD_INCLUDE_DIR AND DHD_LIBRARY)
  SET(DHD_FOUND 0)
  SET(DHD_LIBRARIES)
  SET(DHD_INCLUDE_DIR)
ENDIF(DHD_INCLUDE_DIR  AND DHD_LIBRARY)

# Report the results.
IF(NOT DHD_FOUND)
  SET( DHD_DIR_MESSAGE
       "DHD was not found. Make sure to set DHD_LIBRARY" )
  IF(UNIX)
     SET( DHD_DIR_MESSAGE
          "${DHD_DIR_MESSAGE}, DHD_USB_LIBRARY, DHD_PCISCAN_LIBRARY" )
  ENDIF(UNIX)
  SET( DHD_DIR_MESSAGE
       "${DHD_DIR_MESSAGE} and DHD_INCLUDE_DIR. If you do not have DHD library you will not be able to use the Omega or Delta haptics devices from ForceDimension.")
  IF(DHD_FIND_REQUIRED)
    MESSAGE(FATAL_ERROR "${DHD_DIR_MESSAGE}")
  ELSEIF(NOT DHD_FIND_QUIETLY)
    MESSAGE(STATUS "${DHD_DIR_MESSAGE}")
  ENDIF(DHD_FIND_REQUIRED)
ENDIF(NOT DHD_FOUND)
