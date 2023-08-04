cmake_minimum_required(VERSION 3.8)
project(rfans_driver_ros2)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()
find_package(rfans_driver_msgs REQUIRED)
set(libpcap_LIBRARIES -lpcap)
# find_package(rosidl_default_generators REQUIRED)
# find_package(std_msgs REQUIRED)
# #msg file 
# # set(msg_files
# #   "msg/Packet.msg"
# #   "msg/RfansPacket.msg"
# #   "msg/RfansScan.msg"
# # )
# rosidl_generate_interfaces(${PROJECT_NAME}
#   # ${msg_files}
#   "msg/Packet.msg"
#   "msg/RfansPacket.msg"
#   "msg/RfansScan.msg"
#   "srv/RfansCommand.srv"
#   DEPENDENCIES std_msgs
# )

# ament_export_dependencies(rosidl_default_runtime)
ament_auto_add_library(lidar_calculate src/calculation.cpp)
ament_target_dependencies(lidar_calculate rfans_driver_msgs)

# add_dependencies(lidar_calculate rfans_driver_generate_messages_cpp)
# add_dependencies(lidar_calculate)
# target_link_libraries(lidar_calculate)

#driver_node
ament_auto_add_executable(driver_node
  # src/publisher.cpp
  src/ioapi.cpp
  src/rfans_driver.cpp
  src/ssFrameLib.cpp
  src/bufferDecode.cpp )
target_link_libraries(driver_node lidar_calculate ${libpcap_LIBRARIES})
ament_target_dependencies(driver_node rfans_driver_msgs)
# #calculation_node 
ament_auto_add_executable(calculation_node 
  src/cloud_node.cpp 
  src/ssFrameLib.cpp 
  src/bufferDecode.cpp)
target_link_libraries(calculation_node lidar_calculate)
ament_target_dependencies(calculation_node rfans_driver_msgs)

ament_auto_add_executable(cloud_process src/cloud_process.cpp)
ament_target_dependencies(cloud_process rfans_driver_msgs)
#${${PROJECT_NAME}_EXPORTED_TARGETS})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package()
