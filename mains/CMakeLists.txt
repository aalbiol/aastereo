SET(LIBRARIES stereoUPV cimg_gpiv  utilities)
#demonio_comun
SET(EXTERNAL_LIBS ${SYSTEM_LIBS})

SET(EXTERNAL_LIBS ${EXTERNAL_LIBS} ${OpenCV_LIBS})

include_directories( ../src )

ADD_DEFINITIONS(-DMAIN_UPV_)

#SET (PROGRAMS stereo_shell stereoCalib stereo_evaluate stereo_Energos stereoUD2separate)
#stereo_shell
SET (PROGRAMS  stereoCalib   oneCalib stereo_shell)

foreach(program ${PROGRAMS})
  	      add_executable(${program} ${program}.cpp)
	      target_link_libraries(${program}  ${LIBRARIES})
 	      target_link_libraries(${program}  ${EXTERNAL_LIBS})
endforeach(program)

if (PCL_FOUND)
	ADD_DEFINITIONS(-DHAVE_PCL)
	target_link_libraries(stereo_shell    ${PCL_LIBRARIES})
	target_link_libraries(stereo_shell    cimg_pcl)
endif()

install(TARGETS ${PROGRAMS}   DESTINATION ${CMAKE_INSTALL_BINDIR})
