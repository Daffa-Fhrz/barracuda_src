cmake_minimum_required(VERSION 3.0.2)
project(yolo_omni)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  rospy
  sensor_msgs
  std_msgs
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

# Tambahkan baris untuk menginstal scripts Python jika sudah selesai nanti
# install(PROGRAMS scripts/yolo_node.py DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
