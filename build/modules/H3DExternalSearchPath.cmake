# Contains a function which can be used to get default search paths
# for include and lib directory on windows.

if( WIN32 )
  set( H3D_EXTERNAL_BASE_DIR_NAME "" )
  string( REGEX MATCH "[^ ]+$" vsYearString ${CMAKE_GENERATOR} )
  if( vsYearString )
    if( vsYearString STREQUAL "Win64" )
      string( REGEX REPLACE "([^ ]+) Win64$" "\\1" vsYearString ${CMAKE_GENERATOR} )
      string( REGEX MATCH "[^ ]+$" vsYearString ${vsYearString} )
    endif()
      # 64 bit generator name
    set( H3D_EXTERNAL_BASE_DIR_NAME vs${vsYearString} )
  endif()

  if( CMAKE_CL_64 )
    SET( LIB "lib64" )
  else( CMAKE_CL_64 )
    SET( LIB "lib32" )
  endif( CMAKE_CL_64 )
endif()

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
      list( APPEND h3d_external_base_include_dirs ${h3d_ebd}/include )
      list( APPEND h3d_external_base_lib_dirs ${h3d_ebd}/${LIB} )
      foreach( external_dir_name ${H3D_EXTERNAL_BASE_DIR_NAME} )
        list( APPEND h3d_external_base_include_dirs ${h3d_ebd}/${external_dir_name}/include )
        list( APPEND h3d_external_base_lib_dirs ${h3d_ebd}/${external_dir_name}/${LIB} )
      endforeach( external_dir_name )
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

