cmake_minimum_required(VERSION 3.16)
project(simpack_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)

# 这里定义要生成的消息文件
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/SimpackU.msg"
  "msg/SimpackY.msg"
  DEPENDENCIES builtin_interfaces
)

# 让其他包可以使用本包生成的消息接口
ament_export_dependencies(rosidl_default_runtime)

ament_package()
