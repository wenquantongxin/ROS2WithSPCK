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

if(NOT SPCKRT_LIB)
  message(FATAL_ERROR "Could not find spck_rt library in /opt/Simpack-2021x/run/realtime/linux64")
endif()

if(NOT SPCKRT_INCLUDE_DIR)
  message(FATAL_ERROR "Could not find spck_rt.h in /opt/Simpack-2021x/run/realtime")
endif()

include_directories(
  include
  ${SPCKRT_INCLUDE_DIR}
  ${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_cpp
)

# ====== 第一个可执行文件: simpack_node (含 main_simpack.cpp) ======
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

# ====== 第二个可执行文件: controller_node (含 main_controller.cpp) ======
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

# ====== 安装生成的两个可执行 ======
install(TARGETS simpack_node controller_node
  DESTINATION lib/${PROJECT_NAME}
)

# 如果要安装头文件(如 *.hpp) 也可以:
install(DIRECTORY include/
  DESTINATION include/
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

ament_package()