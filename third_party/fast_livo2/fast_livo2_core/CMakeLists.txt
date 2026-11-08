cmake_minimum_required(VERSION 3.10)
project(fast_livo2_core)

set(CMAKE_BUILD_TYPE "Release")

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread -fexceptions")
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -O0 -g")
# Keep Eigen/PCL alignment ABI compatible with the distro-provided binaries.
# -march=native may enable wider vector alignment than the installed PCL build,
# which corrupts PointCloud containers across shared-library/template calls.
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -O3 -funroll-loops")

add_definitions(-DX86_ARCH)
add_definitions(-DROOT_DIR=\"${CMAKE_CURRENT_SOURCE_DIR}/\")

# OpenMP
find_package(OpenMP QUIET)
if(OpenMP_CXX_FOUND)
  add_compile_options(${OpenMP_CXX_FLAGS})
endif()

# Enable the existing independent per-point LIO residual loop only.
# Keep VIO and every other translation unit serial; no arithmetic changes.
if(NOT OpenMP_CXX_FOUND)
  message(FATAL_ERROR "LIO four-thread candidate requires OpenMP")
endif()
set_source_files_properties(src/voxel_map.cpp PROPERTIES
  COMPILE_DEFINITIONS "MP_EN;MP_PROC_NUM=4"
)

# V16 enables only the two audited VIO patch loops; no global MP_EN reduction.
set(V16_VIO_PATCH_THREADS "4" CACHE STRING "V16 VIO patch team size: 1 or 4")
set(V16_VIO_PATCH_MIN_POINTS "64" CACHE STRING "V16 minimum patch count for OpenMP")
if(NOT V16_VIO_PATCH_THREADS MATCHES "^(1|4)$" OR NOT V16_VIO_PATCH_MIN_POINTS MATCHES "^[0-9]+$")
  message(FATAL_ERROR "Invalid frozen V16 VIO patch configuration")
endif()
set_source_files_properties(src/vio.cpp PROPERTIES
  COMPILE_DEFINITIONS "V16_VIO_PATCH_THREADS=${V16_VIO_PATCH_THREADS};V16_VIO_PATCH_MIN_POINTS=${V16_VIO_PATCH_MIN_POINTS}"
)

# Dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nlohmann_json REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(livox_ros_driver2 REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Sophus REQUIRED)
find_package(vikit_common REQUIRED)
find_package(vikit_ros REQUIRED)
find_package(Boost REQUIRED COMPONENTS thread)
find_package(fmt REQUIRED)

# Sophus compat
if(NOT Sophus_INCLUDE_DIRS AND TARGET Sophus::Sophus)
  get_target_property(Sophus_INCLUDE_DIRS Sophus::Sophus INTERFACE_INCLUDE_DIRECTORIES)
endif()
if(NOT Sophus_LIBRARIES AND TARGET Sophus::Sophus)
  set(Sophus_LIBRARIES Sophus::Sophus)
endif()

# =============================================
# shared library: fast_livo2_core
# =============================================
add_library(fast_livo2_core SHARED
  src/runner.cpp
  src/backend/snapshot_bridge.cpp
  src/LIVMapper.cpp
  src/IMU_Processing.cpp
  src/frame.cpp
  src/preprocess.cpp
  src/vio.cpp
  src/visual_point.cpp
  src/voxel_map.cpp
  src/utils.cpp
)

target_include_directories(fast_livo2_core
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/fast_livo2_core>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/fast_livo2_core/core>
    $<INSTALL_INTERFACE:include/fast_livo2_core>
)

target_include_directories(fast_livo2_core
  SYSTEM PUBLIC
    ${EIGEN3_INCLUDE_DIR}
    ${PCL_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
    ${Sophus_INCLUDE_DIRS}
)
# vikit_common headers: use find_package(vikit_common) + ament_target_dependencies
# instead of raw source path (CMake >=3.28 disallows source paths in INTERFACE includes)

# Direct include path for vikit_ros (camera_loader.h needed at compile time)
# TODO: remove when camera_loader.h is separated from vikit_ros
target_include_directories(fast_livo2_core PRIVATE
    ${vikit_ros_INCLUDE_DIRS}
)

set(deps
  std_msgs
  rclcpp
  geometry_msgs
  nav_msgs
  sensor_msgs
  visualization_msgs
  cv_bridge
  image_transport
  pcl_ros
  pcl_conversions
  tf2_ros
  livox_ros_driver2
  vikit_common
)
ament_target_dependencies(fast_livo2_core ${deps})

target_link_libraries(fast_livo2_core
  nlohmann_json::nlohmann_json
  ${PCL_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${Sophus_LIBRARIES}
  ${Boost_LIBRARIES}
  vikit_common::vikit_common
  vikit_ros::vikit_ros
  fmt::fmt
)
if(OpenMP_CXX_FOUND)
  target_link_libraries(fast_livo2_core OpenMP::OpenMP_CXX)
endif()

# Install
install(TARGETS fast_livo2_core
  EXPORT fast_livo2_core
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(DIRECTORY include/
  DESTINATION include
)

ament_export_targets(fast_livo2_core HAS_LIBRARY_TARGET)
ament_export_dependencies(${deps} nlohmann_json)
ament_package()
