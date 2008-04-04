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
IF( NOT MSVC80 )
  FIND_LIBRARY(wxWidgets_core_LIBRARY NAMES wxmsw28_core   
                                      PATHS $ENV{H3D_EXTERNAL_ROOT}/lib
                                            ../../External/lib )

  FIND_LIBRARY(wxWidgets_base_LIBRARY NAMES wxbase28   
                                       PATHS $ENV{H3D_EXTERNAL_ROOT}/lib
                                             ../../External/lib )
ELSE( NOT MSVC80 )
  FIND_LIBRARY(wxWidgets_core_LIBRARY NAMES wxmsw28_core_vc8   
                                      PATHS $ENV{H3D_EXTERNAL_ROOT}/lib
                                            ../../External/lib )

  FIND_LIBRARY(wxWidgets_base_LIBRARY NAMES wxbase28_vc8
                                       PATHS $ENV{H3D_EXTERNAL_ROOT}/lib
                                             ../../External/lib )
ENDIF( NOT MSVC80 )
MARK_AS_ADVANCED(wxWidgets_base_LIBRARY)
MARK_AS_ADVANCED(wxWidgets_core_LIBRARY)

# Copy the results to the output variables.
IF(wxWidgets_INCLUDE_DIR AND wxWidgets_core_LIBRARY AND wxWidgets_base_LIBRARY)
  SET(wxWidgets_FOUND 1)
  SET(wxWidgets_LIBRARIES ${wxWidgets_core_LIBRARY}
  ${wxWidgets_base_LIBRARY} comctl32 Rpcrt4)
  SET(wxWidgets_INCLUDE_DIRS ${wxWidgets_INCLUDE_DIR})
ELSE(wxWidgets_INCLUDE_DIR AND wxWidgets_core_LIBRARY AND wxWidgets_base_LIBRARY)
  SET(wxWidgets_FOUND 0)
  SET(wxWidgets_LIBRARIES)
  SET(wxWidgets_INCLUDE_DIRS)
ENDIF(wxWidgets_INCLUDE_DIR  AND wxWidgets_core_LIBRARY AND wxWidgets_base_LIBRARY)

