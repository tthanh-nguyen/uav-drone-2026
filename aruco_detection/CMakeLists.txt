cmake_minimum_required(VERSION 3.8)
project(aruco_detection)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)

install(PROGRAMS
  scripts1/aruco_pose_estimation1.py
  scripts1/aruco_node.py
  scripts1/detector.py
  scripts1/calibration.py
  scripts1/transform.py
  scripts1/visualization.py
  scripts1/load_camera.py
  scripts1/maker_localize_node.py
  DESTINATION lib/${PROJECT_NAME}
)

# install(DIRECTORY scripts/camera_calibration/data
#   DESTINATION lib/${PROJECT_NAME}
# )


install(DIRECTORY 
  scripts1/data
  DESTINATION share/${PROJECT_NAME}
)


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

ament_package()
