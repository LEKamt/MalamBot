cmake_minimum_required(VERSION 3.8)
project(camera_detection)

find_package(ament_cmake REQUIRED)
find_package(OpenCV REQUIRED)
find_package(camera_detection_interfaces REQUIRED)

# Compilar el binario panorama_robusto
add_executable(panorama_robusto src/panorama_robusto.cpp)
target_link_libraries(panorama_robusto ${OpenCV_LIBS})

# Instalar el binario
install(TARGETS panorama_robusto
  DESTINATION lib/${PROJECT_NAME}
)

# Instalar el script Python como programa ejecutable
install(PROGRAMS scripts/detection_service.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME detection_service
)

ament_package()