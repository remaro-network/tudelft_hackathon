cmake_minimum_required(VERSION 3.5)
project(tudelft_hackathon)

find_package(ament_cmake REQUIRED COMPONENTS)
find_package(ament_cmake_python REQUIRED COMPONENTS)
find_package(rclpy REQUIRED)
find_package(mavros_msgs REQUIRED)

install(PROGRAMS
  scripts/bluerov_agent.py
  scripts/random_wall_avoidance.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
