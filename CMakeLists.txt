# CMake for Assignment 2
cmake_minimum_required( VERSION 2.8 )

project( Assignment2 )


find_package( PCL 1.3 REQUIRED )

include_directories( .
  /usr/local/include
  /usr/local/include/opencv
  ${PCL_INCLUDE_DIRS} )
link_directories( /usr/local/lib  ${PCL_LIBRARY_DIRS} )
add_definitions( ${PCL_DEFINITIONS} )

set( SRC_FILES grabKinectData grabKinectData.cpp )
set( OpenCV_LIBS opencv_core
  opencv_highgui
  opencv_imgproc
  opencv_calib3d
  opencv_contrib
  opencv_nonfree
  opencv_features2d
  opencv_flann )

#add_executable( grabKinectData ${SRC_FILES} )
#target_link_libraries( grabKinectData ${OpenCV_LIBS} )

#add_executable( Assignment2 Assignment2.cpp Utils.cpp )
#target_link_libraries( Assignment2 ${OpenCV_LIBS} ${PCL_LIBRARIES} )

add_executable( grabPCD grabPCD.cpp )
target_link_libraries( grabPCD ${PCL_LIBRARIES} )

add_executable( showPCD showPCD.cpp )
target_link_libraries( showPCD ${PCL_LIBRARIES} )

add_executable( Reconstruct3D Reconstruct3D.cpp R3D.cpp PCL_Tools/PCL_Tools.cpp )
target_link_libraries( Reconstruct3D ${PCL_LIBRARIES} ${OpenCV_LIBS} )
