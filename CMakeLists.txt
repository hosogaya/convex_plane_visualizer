cmake_minimum_required(VERSION 3.5)
project(convex_plane_visualizer)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(iris_2d REQUIRED)
find_package(convex_plane_msgs REQUIRED)
find_package(convex_plane_converter REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(grid_map_msgs REQUIRED)
find_package(grid_map_core REQUIRED)
find_package(grid_map_ros REQUIRED)
find_package(grid_map_cv REQUIRED)
find_package(autodiff REQUIRED)
find_package(ifopt REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

set(dependencies
  rclcpp
  convex_plane_msgs
  convex_plane_converter
  iris_2d
  visualization_msgs
  grid_map_core
  grid_map_ros
  grid_map_cv
  grid_map_msgs
  ifopt
)

add_library(${PROJECT_NAME} SHARED
  src/convex_plane_visualizer.cpp
)
ament_target_dependencies(${PROJECT_NAME} ${dependencies})
target_link_libraries(${PROJECT_NAME} iris_2d autodiff::autodiff)
target_include_directories(${PROJECT_NAME}
PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
set(node_plugins "")
rclcpp_components_register_nodes(${PROJECT_NAME} "convex_plane::ConvexPlaneVisualizer")
set(node_plugins "${node_plugins}convex_plane::ConvexPlaneVisualizer;$<TARGET_FILE:${PROJECT_NAME}>\n")
ament_export_targets(${PROJECT_NAME} HAS_LIBRARY_TARGET)

ament_export_include_directories(include)
ament_export_dependencies(${dependencies})
ament_export_libraries(
  ${PROJECT_NAME}
)

install(
  TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)
install(
  DIRECTORY include/
  DESTINATION include
)


function(add_visualize_test base_name)
	add_executable(${base_name} test/${base_name}.cpp)
	ament_target_dependencies(${base_name} ${dependencies})
  target_link_libraries(${base_name} ${PROJECT_NAME})
  target_include_directories(${base_name}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    ${EIGEN3_INCLUDE_DIR}
  )
  install(
    TARGETS ${base_name}
    DESTINATION lib/${PROJECT_NAME}
  )
endfunction()

add_visualize_test(test_line_strip)

ament_package()
