cmake_minimum_required(VERSION 3.8)
project(pd_sim)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 23)
endif()

# Add compiler options for GCC and Clang
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Include CPM for package management
include(cmake/get_cpm.cmake)
CPMAddPackage(
    NAME tinyGlTf
    GITHUB_REPOSITORY syoyo/tinygltf
    GIT_TAG v2.9.3
)

# Find dependencies
#   ROS packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
#   other packages
find_package(OpenGL REQUIRED)
find_package(glfw3 3.3 REQUIRED)
find_package(GLEW REQUIRED)
find_package(glm REQUIRED)
find_package(GLU REQUIRED)

# Add executable and source files
add_executable(pd_sim_node 
  src/main.cpp 
  src/loader.cpp 
  src/visualizer.cpp 
  src/trackball.cpp
)

# Link dependencies to the executable
ament_target_dependencies(pd_sim_node rclcpp std_msgs)
target_link_libraries(pd_sim_node tinygltf OpenGL::GL glfw GLEW::GLEW glm::glm GLU)

# Include directories
target_include_directories(pd_sim_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# Install targets and directories
install(TARGETS
  pd_sim_node
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY src/shaders
  DESTINATION share/${PROJECT_NAME}
)

# Define variable for installed shaders directory
set(INSTALLED_SHADERS_DIR "${CMAKE_INSTALL_PREFIX}/share/${PROJECT_NAME}/shaders")

# Pass the shaders directory to visualizer.cpp
target_compile_definitions(pd_sim_node PRIVATE INSTALLED_SHADERS_DIR="${INSTALLED_SHADERS_DIR}")

# Enable testing and linting if BUILD_TESTING is set
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # Skip the linter which checks for copyrights
  # Comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # Skip cpplint (only works in a git repo)
  # Comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
