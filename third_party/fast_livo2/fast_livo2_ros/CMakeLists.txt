cmake_minimum_required(VERSION 3.10)
project(fast_livo2_ros)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread -fexceptions")
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -O3 -funroll-loops")

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(fast_livo2_core REQUIRED)
find_package(vikit_common REQUIRED)
find_package(vikit_ros REQUIRED)
find_package(image_transport REQUIRED)
find_package(Sophus REQUIRED)
find_package(PCL REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Boost REQUIRED COMPONENTS thread)
find_package(Eigen3 REQUIRED)
find_package(fmt REQUIRED)

add_executable(fastlivo_mapping src/main.cpp)

target_include_directories(fastlivo_mapping
  PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/../fast_livo2_core/include/fast_livo2_core/core
)

set(deps
  rclcpp
  fast_livo2_core
  vikit_common
  vikit_ros
  image_transport
)
ament_target_dependencies(fastlivo_mapping ${deps})

target_link_libraries(fastlivo_mapping
  fast_livo2_core::fast_livo2_core
)

install(TARGETS fastlivo_mapping
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/decompress_images.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  config
  launch
  rviz
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
