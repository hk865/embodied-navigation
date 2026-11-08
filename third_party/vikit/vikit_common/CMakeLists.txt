SET(PROJECT_NAME vikit_common)
PROJECT(${PROJECT_NAME})
CMAKE_MINIMUM_REQUIRED (VERSION 3.5)

find_package(ament_cmake REQUIRED)

SET(CMAKE_BUILD_TYPE Release) # Release, RelWithDebInfo
SET(CMAKE_VERBOSE_MAKEFILE OFF)
SET(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${PROJECT_SOURCE_DIR}/CMakeModules/")

# Use portable compiler defaults; do not encode the build host CPU.
SET(CMAKE_CXX_STANDARD 17)
SET(CMAKE_CXX_STANDARD_REQUIRED ON)
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17 -fPIC")
SET(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS} -O3 -fomit-frame-pointer -fno-signed-zeros -fno-math-errno -funroll-loops")

# Add plain cmake packages 
FIND_PACKAGE(OpenCV REQUIRED)
FIND_PACKAGE(Eigen3 REQUIRED)
FIND_PACKAGE(Sophus REQUIRED)
FIND_PACKAGE(fmt REQUIRED)

# Jazzy Sophus compat: Sophus 1.22+ uses cmake targets, not variables
if(NOT Sophus_INCLUDE_DIRS AND TARGET Sophus::Sophus)
  get_target_property(Sophus_INCLUDE_DIRS Sophus::Sophus INTERFACE_INCLUDE_DIRECTORIES)
endif()
if(NOT Sophus_LIBRARIES AND TARGET Sophus::Sophus)
  set(Sophus_LIBRARIES Sophus::Sophus)
endif()

# Include dirs
INCLUDE_DIRECTORIES(
  include
  ${Eigen3_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${Sophus_INCLUDE_DIRS}
)


# Set Sourcefiles
LIST(APPEND SOURCEFILES src/atan_camera.cpp
                        src/omni_camera.cpp
                        src/math_utils.cpp
                        src/vision.cpp
                        src/performance_monitor.cpp
                        src/robust_cost.cpp
                        src/user_input_thread.cpp
                        src/pinhole_camera.cpp
                        src/equidistant_camera.cpp
                        src/polynomial_camera.cpp
                        src/homography.cpp
                        src/img_align.cpp)

# Create vikit library
ADD_LIBRARY(${PROJECT_NAME} SHARED ${SOURCEFILES})
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
TARGET_LINK_LIBRARIES(${PROJECT_NAME}
  ${OpenCV_LIBS}
  ${Sophus_LIBRARIES}
  fmt::fmt)

# Install
install(TARGETS ${PROJECT_NAME}
  EXPORT vikit_common_targets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(DIRECTORY include/
  DESTINATION include
)

ament_export_include_directories(include)
ament_export_libraries(vikit_common)
ament_export_targets(vikit_common_targets HAS_LIBRARY_TARGET)
ament_export_dependencies(Eigen3 OpenCV Sophus fmt)
ament_package()