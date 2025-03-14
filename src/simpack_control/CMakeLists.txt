cmake_minimum_required(VERSION 3.16)
project(simpack_control)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(simpack_interfaces REQUIRED)

# 找 spck_rt.so
find_library(SPCKRT_LIB spck_rt
  HINTS /opt/Simpack-2021x/run/realtime/linux64
)

# 找 spck_rt.h
find_path(SPCKRT_INCLUDE_DIR spck_rt.h
  HINTS /opt/Simpack-2021x/run/realtime
)

# 如果项目中没有集成 nlohmann::json，需要在 package.xml 或系统中安装并让 CMake 找到
# (Ubuntu下: sudo apt-get install nlohmann-json3-dev)
find_package(nlohmann_json REQUIRED)

# 如果没找到 spck_rt 库或头文件，就报错
if(NOT SPCKRT_LIB)
  message(FATAL_ERROR "Could not find spck_rt library in /opt/Simpack-2021x/run/realtime/linux64")
endif()

if(NOT SPCKRT_INCLUDE_DIR)
  message(FATAL_ERROR "Could not find spck_rt.h in /opt/Simpack-2021x/run/realtime")
endif()

# 包含头文件路径
include_directories(
  include
  ${SPCKRT_INCLUDE_DIR}
  ${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_cpp
)

###############################################################################
#                          第 1 个可执行： simpack_node
###############################################################################
add_executable(simpack_node
  src/main_simpack.cpp
  src/SimpackNode.cpp
  src/PIDController.cpp
  src/PiecewiseTrajectory.cpp
)
ament_target_dependencies(simpack_node
  rclcpp
  simpack_interfaces
)
target_link_libraries(simpack_node
  ${SPCKRT_LIB}
  rt
  m
)

###############################################################################
#                     第 2 个可执行： controller_node
###############################################################################
add_executable(controller_node
  src/main_controller.cpp
  src/ControllerNode.cpp
  src/PIDController.cpp
  src/PiecewiseTrajectory.cpp
)
ament_target_dependencies(controller_node
  rclcpp
  simpack_interfaces
)
target_link_libraries(controller_node
  ${SPCKRT_LIB}
  rt
  m
)

###############################################################################
#                 第 3 个可执行： trkrel_udpsender_node 
###############################################################################
add_executable(trkrel_udpsender_node  
  src/TrkRel_UDPSenderNode.cpp
)
ament_target_dependencies(trkrel_udpsender_node  
  rclcpp
  simpack_interfaces
)
# 如果它也需要链接某些库，可在此添加:
# target_link_libraries(trkrel_udpsender_node 
#   ${SPCKRT_LIB}
#   ... 其他库 ...
# )

###############################################################################
#       第 4 个可执行： trkabs_udpsender_node （使用 json 做绝对坐标转换）
###############################################################################
add_executable(trkabs_udpsender_node 
  src/TrkAbs_UDPSenderNode.cpp
)
ament_target_dependencies(trkabs_udpsender_node 
  rclcpp
  simpack_interfaces
)
# 这里用“plain signature”链接 nlohmann_json
target_link_libraries(trkabs_udpsender_node
  nlohmann_json::nlohmann_json
)

###############################################################################
#              安装所有可执行文件到同一目录
###############################################################################
install(TARGETS 
  simpack_node
  controller_node
  trkrel_udpsender_node 
  trkabs_udpsender_node
  DESTINATION lib/${PROJECT_NAME}
)

# 如果需要安装头文件(如 *.hpp) 也可以:
install(DIRECTORY include/
  DESTINATION include/
)

# 如果有 launch 文件夹，也可一并安装
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

ament_package()
