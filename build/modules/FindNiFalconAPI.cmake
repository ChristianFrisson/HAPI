# - Find NiFalconAPI
# Find the native NIFALCONAPI headers and libraries.
#
#  NIFALCONAPI_INCLUDE_DIR -  where to find hdl.h, etc.
#  NIFALCONAPI_LIBRARIES    - List of libraries when using NiFalconAPI.
#  NIFALCONAPI_FOUND        - True if NiFalconAPI found.


# Look for the header file.
FIND_PATH(NIFALCONAPI_INCLUDE_DIR
  NAMES
  falcon/comm/FalconCommLibFTDI.h
  falcon/firmware/FalconFirmwareNovintSDK.h
  falcon/kinematic/FalconKinematicStamper.h
  falcon/util/FalconFirmwareBinaryNvent.h
  falcon/core/FalconDevice.h
  PATHS $ENV{FALCON_SUPPORT}/include
  "/Program Files/NiFalcon/include"
  $ENV{NOVINT_DEVICE_SUPPORT}/include )
MARK_AS_ADVANCED(NIFALCONAPI_INCLUDE_DIR)

# Look for the ftdi library.
FIND_LIBRARY(NIFALCON_COMM_FTDI_LIBRARY
  NAMES nifalcon_comm_libftdi
  PATHS $ENV{FALCON_SUPPORT}/lib
  "/Program Files/NiFalcon/lib"
  $ENV{NOVINT_DEVICE_SUPPORT}/lib)
MARK_AS_ADVANCED(NIFALCON_COMM_FTDI_LIBRARY)

# Look for the usb library.
FIND_LIBRARY(NIFALCON_COMM_USB_LIBRARY
  NAMES nifalcon_comm_libusb
  PATHS $ENV{FALCON_SUPPORT}/lib
  "/Program Files/NiFalcon/lib"
  $ENV{NOVINT_DEVICE_SUPPORT}/lib)
MARK_AS_ADVANCED(NIFALCON_COMM_USB_LIBRARY)

# Look for the ftd2xx library.
FIND_LIBRARY(NIFALCON_COMM_FTD2XX_LIBRARY
  NAMES nifalcon_comm_ftd2xx
  PATHS $ENV{FALCON_SUPPORT}/lib
  "/Program Files/NiFalcon/lib"
  $ENV{NOVINT_DEVICE_SUPPORT}/lib)
MARK_AS_ADVANCED(NIFALCON_COMM_FTD2XX_LIBRARY)

# Set to true if we have any communication library supported
SET( HAVE_NI_FALCON_COMM_LIBRARY NIFALCON_COMM_FTD2XX_LIBRARY OR NIFALCON_COMM_USB_LIBRARY OR NIFALCON_COMM_FTDI_LIBRARY)

# Look for the nifalcon_cpp library.
FIND_LIBRARY(NIFALCON_LIBRARY
  NAMES nifalcon
  PATHS $ENV{FALCON_SUPPORT}/lib
  "/Program Files/NiFalcon/lib"
  $ENV{NOVINT_DEVICE_SUPPORT}/lib)
MARK_AS_ADVANCED(NIFALCON_LIBRARY)

SET(Boost_USE_MULTITHREADED ON)
SET(Boost_USE_STATIC_LIBS ON)

FIND_PACKAGE(Boost REQUIRED COMPONENTS program_options thread)

# Copy the results to the output variables.
IF(NIFALCONAPI_INCLUDE_DIR AND HAVE_NI_FALCON_COMM_LIBRARY AND NIFALCON_LIBRARY)
  SET(NIFALCONAPI_FOUND 1)

  # add the nifalcon library
  SET(NIFALCONAPI_LIBRARIES ${NIFALCON_LIBRARY} )

  # comm libraries, preferred order usb, ftdi, ftd2xx
  IF( NIFALCON_COMM_USB_LIBRARY )
     OPTION(NIFALCON_LIBUSB
           "Use libusb for communication with Falcon device"
           ON)
    SET( HAVE_SET_OPTION 1 )
    SET(NIFALCONAPI_LIBRARIES ${NIFALCONAPI_LIBRARIES} ${NIFALCON_COMM_USB_LIBRARY} )
  ENDIF( NIFALCON_COMM_USB_LIBRARY )
  
  # ftdi library 
  IF( NIFALCON_COMM_FTDI_LIBRARY )
    IF( DEFINED HAVE_SET_OPTION )
      OPTION(NIFALCON_LIBFTDI
             "Use libftdi for communication with Falcon device"
             OFF)
    ELSE( DEFINED HAVE_SET_OPTION )
      OPTION(NIFALCON_LIBFTDI
             "Use libftdi for communication with Falcon device"
             ON)
    ENDIF( DEFINED HAVE_SET_OPTION )
	     
    SET( HAVE_SET_OPTION 1 )
    SET(NIFALCONAPI_LIBRARIES ${NIFALCONAPI_LIBRARIES} ${NIFALCON_COMM_FTDI_LIBRARY} )
  ENDIF( NIFALCON_COMM_FTDI_LIBRARY )

  # ftd2xx library
  IF( NIFALCON_COMM_FTD2XX_LIBRARY )
    IF( DEFINED HAVE_SET_OPTION )
      OPTION(NIFALCON_LIBFTD2XX
             "Use libftd2xx for communication with Falcon device"
    	     OFF )
    ELSE( DEFINED HAVE_SET_OPTION )
      OPTION(NIFALCON_LIBFTD2XX
             "Use libftd2xx for communication with Falcon device"
    	     ON )
    ENDIF( DEFINED HAVE_SET_OPTION )

    SET(NIFALCONAPI_LIBRARIES ${NIFALCONAPI_LIBRARIES} ${NIFALCON_COMM_FTD2XX_LIBRARY} )
  ENDIF( NIFALCON_COMM_FTD2XX_LIBRARY )
    
  SET(NIFALCONAPI_INCLUDE_DIR ${NIFALCONAPI_INCLUDE_DIR} ${Boost_INCLUDE_DIR})
ELSE(NIFALCONAPI_INCLUDE_DIR AND HAVE_NI_FALCON_COMM_LIBRARY AND NIFALCON_LIBRARY)
  SET(NIFALCONAPI_FOUND 0)
  SET(NIFALCONAPI_LIBRARIES)
  SET(NIFALCONAPI_INCLUDE_DIR)
ENDIF(NIFALCONAPI_INCLUDE_DIR  AND HAVE_NI_FALCON_COMM_LIBRARY AND NIFALCON_LIBRARY)
