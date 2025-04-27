cmake_minimum_required(VERSION 3.8)
project(rrt)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(OpenCV REQUIRED)

add_executable(rrt_star_node src/main.cpp src/rrt_star.cpp)  # Correct inclusion of files

target_include_directories(rrt_star_node
  PUBLIC
    include
    ${OpenCV_INCLUDE_DIRS}
)

ament_target_dependencies(rrt_star_node rclcpp OpenCV)

target_link_libraries(rrt_star_node ${OpenCV_LIBS})

install(TARGETS
  rrt_star_node
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)

  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)

  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
