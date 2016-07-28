cmake_minimum_required(VERSION 2.8.3)
project(mrop_gui)

set(CMAKE_CXX_FLAGS "-std=c++0x -fpermissive ${CMAKE_CXX_FLAGS} -Wfatal-errors\ ")

find_package(catkin REQUIRED COMPONENTS rviz)
#find_package(mrop REQUIRED)
find_package(Boost 1.46.1 REQUIRED)
find_package(Qt4 COMPONENTS QtCore QtGui REQUIRED)

include(${QT_USE_FILE})
add_definitions(-DQT_NO_KEYWORDS)

catkin_package(
    INCLUDE_DIRS include /usr/include/OGRE
    LIBRARIES ${PROJECT_NAME}
    CATKIN_DEPENDS joy tf rviz roscpp geometry_msgs
    DEPENDS systemlib QtCore QtGui)

qt4_wrap_cpp(MOC_FILES
#  include/mtracker_panel.h
  include/manual_controller_panel.h
#  include/automatic_controller_panel.h
#  include/obstacle_controller_panel.h
#  include/controls_scaling_panel.h
#  include/reference_generator_panel.h
  include/simulator_panel.h
#  include/state_estimator_panel.h
#  include/data_recorder_panel.h
#  include/path_display.h
)

set(SOURCE_FILES
#  src/mtracker_panel.cpp
  src/manual_controller_panel.cpp
#  src/automatic_controller_panel.cpp
#  src/obstacle_controller_panel.cpp
#  src/controls_scaling_panel.cpp
#  src/reference_generator_panel.cpp
  src/simulator_panel.cpp
#  src/state_estimator_panel.cpp
#  src/data_recorder_panel.cpp
#  src/path_display.cpp
#  src/pose_visual.cpp

#  include/mtracker_panel.h
  include/manual_controller_panel.h
#  include/automatic_controller_panel.h
#  include/obstacle_controller_panel.h
#  include/controls_scaling_panel.h
#  include/reference_generator_panel.h
  include/simulator_panel.h
#  include/state_estimator_panel.h
#  include/data_recorder_panel.h
#  include/path_display.h
#  include/pose_visual.h
  ${MOC_FILES}
)

include_directories(include /usr/include/OGRE ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})# ${mtracker_INCLUDE_DIRS})

set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

add_library(${PROJECT_NAME} ${SOURCE_FILES})
target_link_libraries(${PROJECT_NAME} ${QT_LIBRARIES} ${catkin_LIBRARIES})
add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})
