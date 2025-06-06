cmake_minimum_required(VERSION 3.5)
project(camera_driver)
add_definitions(-DROOT_DIR=\"${CMAKE_CURRENT_SOURCE_DIR}/\")
# Default to C99
if(NOT CMAKE_C_STANDARD)
        set(CMAKE_C_STANDARD 99)
endif()

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
        set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
        add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(OpenCV REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(camera_interfaces REQUIRED)
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
LINK_DIRECTORIES(
        /opt/MVS/lib/64/
        /usr/lib/x86_64-linux-gnu/
)
ament_package()
add_executable(camera_driver_node src/camera_node.cpp src/camera.cpp src/camera_param.cpp)
ament_target_dependencies(camera_driver_node rclcpp sensor_msgs OpenCV camera_interfaces)
target_include_directories(camera_driver_node PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
        /opt/MVS/include/
)
target_link_libraries(camera_driver_node
        MvCameraControl
        yaml-cpp
)
install(TARGETS
        camera_driver_node
        DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY launch yaml
        DESTINATION share/${PROJECT_NAME})