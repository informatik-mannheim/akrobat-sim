cmake_minimum_required(VERSION 2.8.3)
project(hexapod)

find_package(PythonLibs 2.7)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  tf
  gazebo_msgs
)

#${CMAKE_CXX_FLAGS}
set(CMAKE_CXX_FLAGS "-std=c++11 -I/usr/include/python2.7 -lpython2.7")

catkin_package(
   INCLUDE_DIRS include
   CATKIN_DEPENDS roscpp std_msgs tf gazebo_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(akrobat src/akrobat/akrobat_main.cpp src/akrobat/Akrobat.cpp src/akrobat/ControlRandomSampling.cpp src/akrobat/FootConfiguration.cpp src/akrobat/LegSetting.cpp src/akrobat/Trajectory.cpp src/pugixml/pugixml.cpp)

add_executable(jointstate_to_gazebo src/akrobat/JointStateToGazebo.cpp)

target_include_directories(akrobat PRIVATE ${PYTHON_INCLUDE_DIRS})

target_link_libraries(akrobat ${catkin_LIBRARIES})
target_link_libraries(akrobat ${PYTHON_LIBRARIES})
target_link_libraries(jointstate_to_gazebo ${catkin_LIBRARIES})

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(DIRECTORY worlds
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
