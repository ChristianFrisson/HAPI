# Contains a function which can be used to get default search paths
# for include and lib directory on windows.

# This variable can be set after this file is included but before get_external_search_paths_h3d function is called.
# If set to true then then ExternalPath/include/ACKNOWLEDGMENTS file is checked if it corresponds to the current
# version of visual studio. If it is not then the External directory is not added to search path. There is one
# exception though, for visual studio versions below vs2010 the vs2010 year string is looked for.
SET( CHECK_IF_H3D_EXTERNAL_MATCHES_VS_VERSION OFF )

if( MSVC )
  set( H3D_EXTERNAL_BASE_DIR_NAME "" )
  if( MSVC70 )
    set( H3D_EXTERNAL_BASE_DIR_NAME vs2002 ) #This is probably not entirely correct but we won't support this anyways I would assume
  elseif( MSVC71 )
    set( H3D_EXTERNAL_BASE_DIR_NAME vs2003 ) #This is probably not entirely correct but we won't support this anyways I would assume
  elseif( MSVC80 )
    set( H3D_EXTERNAL_BASE_DIR_NAME vs2005 )
  elseif( MSVC90 )
    set( H3D_EXTERNAL_BASE_DIR_NAME vs2008 )
  elseif( MSVC10 )
    set( H3D_EXTERNAL_BASE_DIR_NAME vs2010 )
  elseif( MSVC11 )
    set( H3D_EXTERNAL_BASE_DIR_NAME vs2012 )
  elseif( MSVC12 )
    set( H3D_EXTERNAL_BASE_DIR_NAME vs2013 )
  elseif( MSVC14 )
    set( H3D_EXTERNAL_BASE_DIR_NAME vs2015 )
  endif()

  if( NOT H3D_EXTERNAL_BASE_DIR_NAME ) # fallback for future compilers, this assumes cmake version 3.0 and up.
    string( REGEX MATCH "[^ ]+$" vsYearString ${CMAKE_GENERATOR} )
    if( vsYearString )
      if( vsYearString STREQUAL "Win64" )
        string( REGEX REPLACE "([^ ]+) Win64$" "\\1" vsYearString ${CMAKE_GENERATOR} )
        string( REGEX MATCH "[^ ]+$" vsYearString ${vsYearString} )
      endif()
      set( H3D_EXTERNAL_BASE_DIR_NAME vs${vsYearString} )
    endif()
  endif()

  if( CMAKE_CL_64 )
    SET( LIB "lib64" )
  else( CMAKE_CL_64 )
    SET( LIB "lib32" )
  endif( CMAKE_CL_64 )
endif()

function( check_if_valid_H3D_Win_External arg1 arg2 )
  set( ${arg1} ON PARENT_SCOPE )
  if( CHECK_IF_H3D_EXTERNAL_MATCHES_VS_VERSION )
    set( ${arg1} OFF PARENT_SCOPE )
    if( EXISTS ${arg2}/include/ACKNOWLEDGEMENTS )
      FILE( STRINGS ${arg2}/include/ACKNOWLEDGEMENTS the_first_line LIMIT_COUNT 1 REGEX "[-][-][-][-] Compiled for ${H3D_EXTERNAL_BASE_DIR_NAME} [-][-][-][-]" ) # Check if first line contains the correct vsx string.
      if( the_first_line )
        set( ${arg1} ON PARENT_SCOPE )
      endif()
    endif()
  endif()
endfunction()

# arg1 Will contain search path for the include directories.
# arg2 Will contain search path for library directories.
# arg3 Should contain FindXX.cmake module path or empty,
# or basically additional base paths External is expected to possibly exists
# like this relative to this path ../../../External
function( get_external_search_paths_h3d arg1 arg2 arg3 )
  if( WIN32 )
    set( h3d_external_base_dirs $ENV{H3D_EXTERNAL_ROOT}
                                $ENV{H3D_ROOT}/../External
                                ../../External
                                ${arg3}/../../../External )
    set( h3d_external_base_include_dirs "" )
    set( h3d_external_base_lib_dirs "" )
    foreach( h3d_ebd ${h3d_external_base_dirs} )
      check_if_valid_H3D_Win_External( add_dir ${h3d_ebd} )
      if( add_dir )
        list( APPEND h3d_external_base_include_dirs ${h3d_ebd}/include )
        list( APPEND h3d_external_base_lib_dirs ${h3d_ebd}/${LIB} )
      endif()
      if( H3D_EXTERNAL_BASE_DIR_NAME )
        foreach( external_dir_name ${H3D_EXTERNAL_BASE_DIR_NAME} )
          check_if_valid_H3D_Win_External( add_dir ${h3d_ebd}/${external_dir_name} )
          if( add_dir )
            list( APPEND h3d_external_base_include_dirs ${h3d_ebd}/${external_dir_name}/include )
            list( APPEND h3d_external_base_lib_dirs ${h3d_ebd}/${external_dir_name}/${LIB} )
          endif()
        endforeach( external_dir_name )
      endif()
    endforeach( h3d_ebd )
    
    set( tmp_include_dir_output "" )
    foreach( h3d_base_include_dir ${h3d_external_base_include_dirs} )
      list( APPEND tmp_include_dir_output ${h3d_base_include_dir} )
      foreach( f ${ARGN} )
        list( APPEND tmp_include_dir_output ${h3d_base_include_dir}/${f} )
      endforeach(f)
    endforeach( h3d_base_include_dir )
    set( ${arg1} ${tmp_include_dir_output} PARENT_SCOPE )
    
    set( tmp_lib_dir_output "" )
    foreach( h3d_base_lib_dir ${h3d_external_base_lib_dirs} )
      list( APPEND tmp_lib_dir_output ${h3d_base_lib_dir} )
      foreach( f ${ARGN} )
        list( APPEND tmp_lib_dir_output ${h3d_base_lib_dir}/${f} )
      endforeach(f)
    endforeach( h3d_base_lib_dir )
    set( ${arg2} ${tmp_lib_dir_output} PARENT_SCOPE )
  endif()
endfunction()

