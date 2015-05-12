# - Find DHD
# Find the native DHD headers and libraries.
#
#  DHD_INCLUDE_DIR -  where to find DHD headers
#  DHD_LIBRARIES    - List of libraries when using DHD.
#  DHD_FOUND        - True if DHD found.

include( H3DExternalSearchPath )
GET_FILENAME_COMPONENT( module_file_path ${CMAKE_CURRENT_LIST_FILE} PATH )
get_external_search_paths_h3d( module_include_search_paths module_lib_search_paths ${module_file_path} "DHD-API" )

# Look for the header file.
FIND_PATH(DHD_INCLUDE_DIR NAMES dhdc.h 
                          PATHS ${module_include_search_paths}
                          DOC "Path in which the file dhdc.h is located." )
MARK_AS_ADVANCED(DHD_INCLUDE_DIR)


# Look for the library.
IF(WIN32)
  FIND_LIBRARY(DHD_LIBRARY NAMES dhdms dhdms64
                           PATHS ${module_lib_search_paths}
                           DOC "Path to dhdms library." )
ELSE(WIN32)
  FIND_LIBRARY(DHD_LIBRARY NAMES dhd
                           PATHS ${module_lib_search_paths}
                           DOC "Path to dhd library." )

  IF(APPLE)
    FIND_LIBRARY( DHD_IOKIT_LIBRARY NAMES IOKit
                  DOC "Path to IOKit library." )
    FIND_LIBRARY( DHD_COREFOUNDATION_LIBRARY NAMES CoreFoundation
                  DOC "Path to CoreFoundation library." )
    MARK_AS_ADVANCED(DHD_IOKIT_LIBRARY)
    MARK_AS_ADVANCED(DHD_COREFOUNDATION_LIBRARY)
  ELSE(APPLE)
    IF(UNIX)
      FIND_LIBRARY( DHD_USB_LIBRARY NAMES usb
                    DOC "Path to usb library." )
      FIND_LIBRARY( DHD_PCISCAN_LIBRARY NAMES pciscan 
                    DOC "Path to pciscan library." )
      MARK_AS_ADVANCED(DHD_USB_LIBRARY)
      MARK_AS_ADVANCED(DHD_PCISCAN_LIBRARY)
    ENDIF(UNIX)
  ENDIF(APPLE)
ENDIF(WIN32)
MARK_AS_ADVANCED(DHD_LIBRARY)


# Copy the results to the output variables.
IF(DHD_INCLUDE_DIR AND DHD_LIBRARY)
  SET(DHD_FOUND 1)
  SET(DHD_LIBRARIES ${DHD_LIBRARY})
  SET(DHD_INCLUDE_DIR ${DHD_INCLUDE_DIR})
  IF(APPLE)
    IF(DHD_IOKIT_LIBRARY AND DHD_COREFOUNDATION_LIBRARY)
        SET(DHD_LIBRARIES ${DHD_LIBRARIES} ${DHD_IOKIT_LIBRARY} ${DHD_COREFOUNDATION_LIBRARY})
      ELSE(DHD_IOKIT_LIBRARY AND DHD_COREFOUNDATION_LIBRARY)
        SET(DHD_FOUND 0)
        SET(DHD_LIBRARIES)
        SET(DHD_INCLUDE_DIR)
      ENDIF(DHD_IOKIT_LIBRARY AND DHD_COREFOUNDATION_LIBRARY)
  ELSE(APPLE)
    IF(UNIX)
      IF(DHD_USB_LIBRARY AND DHD_PCISCAN_LIBRARY)
        SET(DHD_LIBRARIES ${DHD_LIBRARIES} ${DHD_USB_LIBRARY} ${DHD_PCISCAN_LIBRARY})
      ELSE(DHD_USB_LIBRARY AND DHD_PCISCAN_LIBRARY)
        SET(DHD_FOUND 0)
        SET(DHD_LIBRARIES)
        SET(DHD_INCLUDE_DIR)
      ENDIF(DHD_USB_LIBRARY AND DHD_PCISCAN_LIBRARY)
    ENDIF(UNIX)
  ENDIF(APPLE)
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
