SET(SOURCE_FILES CimgStereo.cpp CimgStereoCalib.cpp stereo_xmlconfig.cpp CimgStereoDisplay.cpp CimgOneCalib.cpp  distort.cpp)
#stereo_src_Energos.cpp

#IF (OpenCV_VERSION VERSION_GREATER 2.3)
#	SET(SOURCE_FILES ${SOURCE_FILES} stereovar_upv.cpp)
#ENDIF()

ADD_LIBRARY(stereoUPV ${SOURCE_FILES})
install(TARGETS stereoUPV DESTINATION ${CMAKE_INSTALL_LIBDIR})



