cmake_minimum_required(VERSION 3.2)

find_package(PkgConfig REQUIRED)


PROJECT(UPVSOFT3)

set(HOME $ENV{HOME})

set (CMAKE_CXX_STANDARD 11)
add_compile_options(-Wno-deprecated)

set(CMAKE_LIBRARY_PATH ${CMAKE_LIBRARY_PATH}  /usr/local/share/cmake /usr/local/share )
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/cmake/Modules/")

if(APPLE)
#Para que encuentre OpenCV de ports
#set(CMAKE_PREFIX_PATH  "/opt/local/lib/cmake")
set (CMAKE_PREFIX_PATH ${CMAKE_PREFIX_PATH} /opt/local/share)
endif(APPLE)

message ("MAKE_MODULE_PATH ${CMAKE_MODULE_PATH}")
message("CMAKE_PREFIX_PATH ${CMAKE_PREFIX_PATH}")
set(UPVSOFT_PATH ${PROJECT_SOURCE_DIR})
#set (CMAKE_BUILD_TYPE Release)
SET(CMAKE_INSTALL_LIBDIR ${HOME}/lib )

if(CMAKE_BUILD_TYPE STREQUAL Debug)
		  SET(CMAKE_INSTALL_LIBDIR ${HOME}/lib)
elseif(CMAKE_BUILD_TYPE STREQUAL Release)
		  SET(CMAKE_INSTALL_LIBDIR ${HOME}/lib )
elseif(CMAKE_BUILD_TYPE STREQUAL RelWithDebInfo)
		  SET(CMAKE_INSTALL_LIBDIR ${HOME}/lib )
elseif(CMAKE_BUILD_TYPE STREQUAL MinSizeRel)
		  SET(CMAKE_INSTALL_LIBDIR ${HOME}/lib )
endif(CMAKE_BUILD_TYPE STREQUAL Debug)


if (DEFINED PROFILING)
   SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -p")
endif (DEFINED PROFILING)

SET(CMAKE_INSTALL_BINDIR ${HOME}/bin)

FIND_PACKAGE(Qt4 COMPONENTS  QtCore )
FIND_PACKAGE(JPEG REQUIRED)
FIND_PACKAGE(PNG)
FIND_PACKAGE(X11 REQUIRED)
FIND_PACKAGE(TIFF REQUIRED)
FIND_PACKAGE(LibXml2 REQUIRED)
FIND_PACKAGE(Threads REQUIRED)
FIND_PACKAGE(ZLIB)
FIND_PACKAGE(LAPACK)
FIND_PACKAGE(BLAS)
#FIND_PACKAGE(BOOST)
if (BLAS_FOUND)
#  FIND_PACKAGE(CBLAS)
endif()


#PKG_CHECK_MODULES(FFTW3 fftw3)

PKG_CHECK_MODULES(EIGEN3 eigen3)
IF(EIGEN3_FOUND)
	include_directories(${EIGEN3_INCLUDE_DIRS})
ENDIF(EIGEN3_FOUND)

include_directories(${X11_INCLUDE_DIR})
include_directories(${LIBXML2_INCLUDE_DIR})
#include_directories(${TIFF_INCLUDE_DIR})

#pkg_check_modules(SDL sdl)

link_directories(${X11_LIB_DIRS})

get_filename_component(TIFF_LIB_DIRS ${TIFF_LIBRARIES} PATH)
link_directories(${TIFF_LIB_DIRS})
message ("TIFF LIB Directory = " ${TIFF_LIB_DIRS})

get_filename_component(JPEG_LIB_DIRS ${JPEG_LIBRARIES} PATH)
link_directories(${JPEG_LIB_DIRS})

link_directories(${LIBXML2_LIB_DIRS})

SET(SYSTEM_LIBS ${TIFF_LIBRARIES} ${JPEG_LIBRARIES} ${X11_LIBRARIES} ${LIBXML2_LIBRARIES} ${CMAKE_THREAD_LIBS_INIT})

#PKG_CHECK_MODULES(OPENCV opencv)
FIND_PACKAGE(OpenCV )

SET(OPENCV_FOUND ${OpenCV_FOUND})

#no hara falta poner este directorio para link!
#SET(OPENCV_LIBRARY_DIRS ${OpenCV_LIB_DIR})
#SET(OPENCV_INCLUDE_DIRS ${OpenCV_INCLUDE_DIRS})
#SET(OPENCV_LIBRARIES ${OpenCV_LIBS})
SET(OPENCV_VERSION ${OpenCV_VERSION})



IF (OPENCV_FOUND)
    ADD_DEFINITIONS(-DHAVE_OPENCV -DOPENCV)
    INCLUDE_DIRECTORIES(${OpenCV_INCLUDE_DIRS})

     IF(OPENCV_VERSION LESS 2.3)
       ADD_DEFINITIONS(-DOLD_STEREO_OPENCV)
     ENDIF()
ENDIF()



if  (FFT3_FOUND)
	message("FFT3 Found")
	SET(CIMG_FFTW3_FLAGS -Dcimg_use_fftw3)

	include_directories(${FFTW3_INCLUDE_DIR})
	SET(SYSTEM_LIBS ${SYSTEM_LIBS} ${FFTW3_LIBRARIES} )
	SET(CIMG_CFLAGS  "${CIMG_CFLAGS} ${CIMG_FFT3_FLAGS}")
endif()

# Flags to enable fast image display, using the XSHM library.
SET(CIMG_XSHM_CCFLAGS  -Dcimg_use_xshm)

# Flags to enable screen mode switching, using the XRandr library.
SET(CIMG_XRANDR_CCFLAGS  -Dcimg_use_xrandr)

# Flags to enable native support for JPEG image files, using the JPEG library.
SET(CIMG_JPEG_CCFLAGS  -Dcimg_use_jpeg)

# Flags to enable native support for TIFF image files, using the TIFF library.
SET(CIMG_TIFF_CCFLAGS  -Dcimg_use_tiff)

SET(CIMG_ZLIB_FLAGS -Dcimg_use_zlib)
SET(CIMG_PNG_CCFLAGS  -Dcimg_use_png)


  # Flags to enable native support of upvsoft.
SET(CIMG_UPVSOFT_CCFLAGS  -Dcimg_use_upvsoft)



if ( WIN32 )
  SET(CIMG_CFLAGS  ${CIMG_UPVSOFT_CCFLAGS})
  SET(SYSTEM_LIBS gdi32  ${LIBXML2_LIBRARIES} )
else()
     SET(CIMG_CFLAGS "${CIMG_JPEG_CCFLAGS} ${CIMG_TIFF_CCFLAGS}  ${CIMG_UPVSOFT_CCFLAGS}")
     SET(CIMG_CFLAGS "${CIMG_CFLAGS} ${CIMG_X11_CCFLAGS}")
     SET(SYSTEM_LIBS ${TIFF_LIBRARIES} ${JPEG_LIBRARIES} ${X11_LIBRARIES} ${LIBXML2_LIBRARIES} ${CMAKE_THREAD_LIBS_INIT})
endif()




if  (ZLIB_FOUND)
	include_directories(${ZLIB_INCLUDE_DIR})
	SET(SYSTEM_LIBS ${SYSTEM_LIBS} ${ZLIB_LIBRARIES})
	SET(CIMG_CFLAGS  "${CIMG_CFLAGS} ${CIMG_ZLIB_FLAGS}")
endif()

if  (PNG_FOUND)
	include_directories(${PNG_INCLUDE_DIR})
	SET(SYSTEM_LIBS ${SYSTEM_LIBS} ${PNG_LIBRARIES})
	SET(CIMG_CFLAGS  "${CIMG_CFLAGS} ${CIMG_PNG_FLAGS}")
endif()

IF (OpenCV_FOUND)
# Flags to enable native support of opencv
SET(CIMG_OPENCV_CCFLAGS  -Dcimg_use_opencv)
#	SET(CIMG_CFLAGS  "${CIMG_CFLAGS} ${CIMG_OPENCV_CCFLAGS}")
ENDIF()



ADD_DEFINITIONS(-D_FILE_OFFSET_BITS=64 -D_LARGEFILE_SOURCE -D_GNU_SOURCE  -D_REENTRANT)



ADD_DEFINITIONS(-D_FILE_OFFSET_BITS=64 -D_LARGEFILE_SOURCE -D_GNU_SOURCE  -D_REENTRANT)


ADD_DEFINITIONS(-Wno-unknown-pragmas)



if(APPLE)
 #        SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Qunused-arguments")
endif(APPLE)

if(NOT APPLE)
SET(CIMG_CFLAGS "${CIMG_CFLAGS} ${CIMG_XSHM_CCFLAGS} ${CIMG_XRANDR_CCFLAGS}" )
  if(NOT WIN32)
  	SET(SYSTEM_LIBS ${SYSTEM_LIBS} Xext Xrandr)
        SET(SYSTEM_LIBS ${SYSTEM_LIBS} pthread)
	#SUBDIRS(linux)
  endif(NOT WIN32)
endif(NOT APPLE)

if(WITH_PCL STREQUAL On)
	find_package(PCL)
endif(WITH_PCL STREQUAL On)


add_definitions(${CIMG_CFLAGS})

include_directories(${UPVSOFT_PATH}/cimg/src)
include_directories(${UPVSOFT_PATH}/victor/include)
#SUBDIRS(victor svm)
SUBDIRS( svm  cimg   corners dimensionality_reduction)
#SUBDIRS(      dimensionality_reduction )
#SUBDIRS( interfaces)
subdirs(misc)

#subdirs(surf)

#interfaces
if (OpenCV_FOUND)
		subdirs(stereo_jmmossi)
		#SUBDIRS(   features_opencv)
		#subdirs(clasificadores_opencv)
		subdirs( demos_opencv )

endif(OpenCV_FOUND)
