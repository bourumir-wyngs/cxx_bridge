cmake_minimum_required(VERSION 3.8)
project(cxx_bridge)

set(CMAKE_CXX_STANDARD 23)

# Compiler flags
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ROS 2 dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)

# Find Poco libraries
find_package(Poco REQUIRED COMPONENTS Net Foundation)

# Protobuf support
find_package(Protobuf REQUIRED)

find_package(nlohmann_json REQUIRED)

# Include Protobuf and gRPC headers
include_directories(${Protobuf_INCLUDE_DIRS})

# Add .proto files
set(PROTO_FILES
        ${CMAKE_CURRENT_SOURCE_DIR}/proto/pose_array.proto
        ${CMAKE_CURRENT_SOURCE_DIR}/proto/joint_trajectory_dof6.proto
)

# Generate Protobuf C++ code
protobuf_generate_cpp(PROTO_SRCS PROTO_HDRS ${PROTO_FILES})

# Include generated and custom headers
include_directories(${CMAKE_CURRENT_BINARY_DIR})
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)

# Add gRPC and external library paths, if applicable
include_directories(/usr/include)
include_directories(/usr/include/grpcpp)

# Define executables
add_executable(main src/main.cpp ${PROTO_SRCS})

# Link libraries for the first executable (main)
target_link_libraries(main
        ${Protobuf_LIBRARIES}        # Link Protobuf
        Poco::Net                    # Link Poco Net
        Poco::Foundation             # Link Poco Foundation
        nlohmann_json::nlohmann_json
)

# Define protobuf_server executable
add_executable(protobuf_server src/protobuf_server.cpp src/ros_sender.cpp ${PROTO_SRCS})

# Link Protobuf and ROS 2 libraries to protobuf_server
target_link_libraries(protobuf_server
        ${Protobuf_LIBRARIES}
        Poco::Net
        Poco::Foundation
)

target_include_directories(protobuf_server PUBLIC
)

ament_target_dependencies(protobuf_server
        rclcpp
        std_msgs
        geometry_msgs
        trajectory_msgs
        rcl_interfaces
)

# Install targets
install(TARGETS main protobuf_server
        DESTINATION lib/${PROJECT_NAME})

# Lint and testing configuration
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # The following line skips the linter which checks for copyrights
  # Comment the line when a copyright and license are added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # The following line skips cpplint (only works in a git repo)
  # Remove this when the package is in a git repo, and copyright/license is added
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# Ament package finalization
ament_package()