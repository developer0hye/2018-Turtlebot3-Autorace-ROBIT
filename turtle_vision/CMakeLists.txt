cmake_minimum_required(VERSION 2.8.0)
project(turtle_vision)

find_package(catkin REQUIRED COMPONENTS qt_build roscpp cv_bridge image_transport rospy sensor_msgs std_msgs message_generation)

set(OpenCV_INCLUDE_DIRS
  #/usr/local/include
  /usr/local/include/opencv2
)

set(OpenCV_LIB_DIR
  /usr/local/lib
)

set(OpenCV_LIBS
  opencv_core
  opencv_highgui
  opencv_imgcodecs
  opencv_videoio
  opencv_imgproc
  opencv_features2d
  opencv_xfeatures2d
  opencv_flann
  opencv_calib3d
)

set(Darknet_Pjreddie_INCLUDE_DIR
  /home/turtle1/Libs/darknet_AlexeyAB/src/
)

set(Darknet_Pjreddie_LIB_DIR
  /home/turtle1/Libs/darknet_AlexeyAB/
)

set(Darknet_Pjreddie_LIBS
    darknet
)

include_directories(${catkin_INCLUDE_DIRS}  ${OpenCV_INCLUDE_DIRS}  ${Darknet_Pjreddie_INCLUDE_DIR})

link_directories(${OpenCV_LIB_DIR})
link_directories(${Darknet_Pjreddie_LIB_DIR})

add_message_files(
  FILES
  vision_msg.msg
)
generate_messages(
   DEPENDENCIES
   std_msgs
)

catkin_package()

rosbuild_prepare_qt4(QtCore QtGui) 

file(GLOB QT_FORMS RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} ui/*.ui)
file(GLOB QT_RESOURCES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} resources/*.qrc)
file(GLOB_RECURSE QT_MOC RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} FOLLOW_SYMLINKS include/turtle_vision/*.hpp)

QT4_ADD_RESOURCES(QT_RESOURCES_CPP ${QT_RESOURCES})
QT4_WRAP_UI(QT_FORMS_HPP ${QT_FORMS})
QT4_WRAP_CPP(QT_MOC_HPP ${QT_MOC})

file(GLOB_RECURSE QT_SOURCES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} FOLLOW_SYMLINKS src/*.cpp)

add_executable(turtle_vision ${QT_SOURCES} ${QT_RESOURCES_CPP} ${QT_FORMS_HPP} ${QT_MOC_HPP})
target_link_libraries(turtle_vision ${QT_LIBRARIES} ${catkin_LIBRARIES} ${OpenCV_LIBS} ${Darknet_Pjreddie_LIBS})
install(TARGETS turtle_vision RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")
