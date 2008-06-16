# - Find wxWidgets
# Find the native wxWidgets headers and libraries.
#
#  wxWidgets_INCLUDE_DIR -  where to find WxWidgets headers
#  wxWidgets_LIBRARIES    - List of libraries when using WxWidgets.
#  wxWidgets_FOUND        - True if WxWidgets found.


# Look for the header file.
FIND_PATH(wxWidgets_INCLUDE_DIR NAMES wx/wx.h 
                                PATHS $ENV{H3D_EXTERNAL_ROOT}/include
                                      ../../External/include )
MARK_AS_ADVANCED(wxWidgets_INCLUDE_DIR)

# Look for the library.
IF( MSVC70 OR MSVC71 )
  FIND_LIBRARY(wxWidgets_core_LIBRARY NAMES wxmsw28_core   
                                      PATHS $ENV{H3D_EXTERNAL_ROOT}/lib
                                            ../../External/lib )

  FIND_LIBRARY(wxWidgets_base_LIBRARY NAMES wxbase28   
                                       PATHS $ENV{H3D_EXTERNAL_ROOT}/lib
                                             ../../External/lib )
  IF(WXWINDOWS_USE_GL)
    FIND_LIBRARY(wxWidgets_gl_LIBRARY NAMES wxmsw28_gl   
                                      PATHS $ENV{H3D_EXTERNAL_ROOT}/lib
                                            ../../External/lib )

    FIND_LIBRARY(wxWidgets_adv_LIBRARY NAMES wxmsw28_adv   
                                       PATHS $ENV{H3D_EXTERNAL_ROOT}/lib
                                             ../../External/lib )
  ENDIF(WXWINDOWS_USE_GL)
ELSE( MSVC70 OR MSVC71 )
  FIND_LIBRARY(wxWidgets_core_LIBRARY NAMES wxmsw28_core_vc8   
                                      PATHS $ENV{H3D_EXTERNAL_ROOT}/lib
                                            ../../External/lib )

  FIND_LIBRARY(wxWidgets_base_LIBRARY NAMES wxbase28_vc8
                                       PATHS $ENV{H3D_EXTERNAL_ROOT}/lib
                                             ../../External/lib )
  IF(WXWINDOWS_USE_GL)
    FIND_LIBRARY(wxWidgets_gl_LIBRARY NAMES wxmsw28_gl_vc8
                                      PATHS $ENV{H3D_EXTERNAL_ROOT}/lib
                                            ../../External/lib )

    FIND_LIBRARY(wxWidgets_adv_LIBRARY NAMES wxmsw28_adv_vc8
                                       PATHS $ENV{H3D_EXTERNAL_ROOT}/lib
                                             ../../External/lib )
  ENDIF(WXWINDOWS_USE_GL)
ENDIF( MSVC70 OR MSVC71 )
MARK_AS_ADVANCED(wxWidgets_base_LIBRARY)
MARK_AS_ADVANCED(wxWidgets_core_LIBRARY)
IF(WXWINDOWS_USE_GL)
  MARK_AS_ADVANCED(wxWidgets_gl_LIBRARY)
  MARK_AS_ADVANCED(wxWidgets_adv_LIBRARY)
ENDIF(WXWINDOWS_USE_GL)

# Copy the results to the output variables.
IF(wxWidgets_INCLUDE_DIR AND wxWidgets_core_LIBRARY AND wxWidgets_base_LIBRARY)
  IF( WXWINDOWS_USE_GL )
    IF( wxWidgets_gl_LIBRARY AND wxWidgets_adv_LIBRARY )
      SET(wxWidgets_FOUND 1)
      SET( wxWidgets_LIBRARIES ${wxWidgets_core_LIBRARY}
           ${wxWidgets_base_LIBRARY} ${wxWidgets_gl_LIBRARY} ${wxWidgets_adv_LIBRARY} comctl32 Rpcrt4)
      SET(wxWidgets_INCLUDE_DIR ${wxWidgets_INCLUDE_DIR})
    ELSE( wxWidgets_gl_LIBRARY AND wxWidgets_adv_LIBRARY )
      SET(wxWidgets_FOUND 0)
      SET(wxWidgets_LIBRARIES)
      SET(wxWidgets_INCLUDE_DIR)
    ENDIF( wxWidgets_gl_LIBRARY AND wxWidgets_adv_LIBRARY )
  ELSE( WXWINDOWS_USE_GL )
    SET(wxWidgets_FOUND 1)
    SET( wxWidgets_LIBRARIES ${wxWidgets_core_LIBRARY}
         ${wxWidgets_base_LIBRARY} comctl32 Rpcrt4)
    SET(wxWidgets_INCLUDE_DIR ${wxWidgets_INCLUDE_DIR})
  ENDIF( WXWINDOWS_USE_GL )
ELSE(wxWidgets_INCLUDE_DIR AND wxWidgets_core_LIBRARY AND wxWidgets_base_LIBRARY)
  SET(wxWidgets_FOUND 0)
  SET(wxWidgets_LIBRARIES)
  SET(wxWidgets_INCLUDE_DIR)
ENDIF(wxWidgets_INCLUDE_DIR  AND wxWidgets_core_LIBRARY AND wxWidgets_base_LIBRARY)

# Report the results.
IF(NOT wxWidgets_FOUND)
  SET(wxWidgets_DIR_MESSAGE
    "WxWidgets was not found. Make sure wxWidgets_core_LIBRARY, wxWidgets_base_LIBRARY")
  IF( WXWINDOWS_USE_GL )
    SET( wxWidgets_DIR_MESSAGE "${wxWidgets_DIR_MESSAGE}, wxWidgets_gl_LIBRARY, wxWidgets_adv_LIBRARY")
  ENDIF( WXWINDOWS_USE_GL )
  SET( wxWidgets_DIR_MESSAGE "${wxWidgets_DIR_MESSAGE} and wxWidgets_INCLUDE_DIR are set to where you have your wxWidgets header and lib files.")
  IF(wxWidgets_FIND_REQUIRED)
      MESSAGE(FATAL_ERROR "${wxWidgets_DIR_MESSAGE}")
  ELSEIF(NOT wxWidgets_FIND_QUIETLY)
    MESSAGE(STATUS "${wxWidgets_DIR_MESSAGE}")
  ENDIF(wxWidgets_FIND_REQUIRED)
ENDIF(NOT wxWidgets_FOUND)
