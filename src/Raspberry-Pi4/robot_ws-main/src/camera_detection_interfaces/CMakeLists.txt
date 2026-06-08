cmake_minimum_required(VERSION 3.8)
project(camera_detection_interfaces)

find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/DetectPipe.srv"
)

ament_package()