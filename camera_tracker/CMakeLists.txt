cmake_minimum_required(VERSION 3.5)
project(camera_tracker)

# Подключаем зависимости
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(cv_bridge REQUIRED)

# Указываем директорию с Python скриптами
ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
  camera_tracker/tracker_node.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
