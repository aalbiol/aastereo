SET(SOURCE_FILES CimgStereo.cpp CimgStereoCalib.cpp stereo_xmlconfig.cpp CimgStereoDisplay.cpp CimgOneCalib.cpp  distort.cpp)


ADD_LIBRARY(stereoUPV ${SOURCE_FILES})
#install(TARGETS stereoUPV DESTINATION ${CMAKE_INSTALL_LIBDIR})

ADD_LIBRARY(utilities string_utilities.cpp file_utilities.cpp  xml_utilities.cpp )



