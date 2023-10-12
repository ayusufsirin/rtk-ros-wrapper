cmake_minimum_required(VERSION 3.0.2)
project(rtk_wrapper)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  rospy
  sensor_msgs
  std_msgs
)

catkin_package(

)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS
  scripts/navsat_to_kml.py
  scripts/navsat_scaler.py
  scripts/RTK_ROS.py
  scripts/ntrip_client.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# catkin build -DPYTHON_EXECUTABLE=/usr/bin/python3 -DPYTHON_INCLUDE_DIR=/usr/include/python3.7m