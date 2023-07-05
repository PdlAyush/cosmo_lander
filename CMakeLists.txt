cmake_minimum_required(VERSION 3.5)
project(lander)


find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)

set(msg_files
  "msg/Coordinate.msg"
  "msg/RegionOfInterest.msg"
  "msg/PointOfInterest.msg"
)

set(srv_files
  "srv/GetPOI.srv"
  "srv/GetROI.srv"
  "srv/SetPOI.srv"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  ${srv_files}
)

install(PROGRAMS
  lander/lander_services.py
  DESTINATION lib/${PROJECT_NAME}/lander
)

install(PROGRAMS
  lander/lander_server.py
  DESTINATION lib/${PROJECT_NAME}/lander
)

install(
  FILES
  lander/poi.json
  lander/roi.json
  DESTINATION lib/${PROJECT_NAME}/lander
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()

