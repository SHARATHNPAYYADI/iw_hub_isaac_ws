cmake_minimum_required(VERSION 3.8)
project(iw_hub_pick_place_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/RunMission.srv"
  "srv/AddWaypoint.srv"
  "srv/ListWaypoints.srv"
  "srv/DriveStraight.srv"
  "srv/Rotate.srv"
)

ament_package()
